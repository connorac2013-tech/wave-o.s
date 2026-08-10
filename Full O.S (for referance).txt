/*
ESP32-S3-Touch-LCD-3.5 (Waveshare) — Wave OS AIO V5.0 Multi-Core
Features: Persistent Settings, Themed Tiles, Centered WiFi Keyboard,
Scrollable WiFi List, Splash, PIN, I2S Sound, Camera Privacy Lock,
Sketch Color/Brush Toolbar, Notepad App, True Lowercase Support,
Camera Auto-Rotate, Auto-Connect WiFi at Boot, RESTART Button,
POWER MENU (Shutdown, Hibernate, Sleep, Restart),
RETRO RACING GAME with Gyro Steering + On-Screen Arrows & Local Refresh,
CLEAR GHOSTING APP (30-Min Lockout), START MENU (W),
SCREEN SAVER (Bouncy Smile, DVD Logo, Bouncy W),
IP EXPLORER (Network Scanner & Web Viewer with Progress Bar),
DRIVER RACING GAME (Race NPC Drivers),
FILE SAVE/OPEN/DELETE/RENAME (.esptxt / .espimg via SD Card),
SCREENSHOT (Hold BOOT 2 sec),
CAMERA TOUCH FIX (I2C bus re-init after camera),

NEW IN V5.0: TRUE MULTI-CORE OS ARCHITECTURE
- Core 0 (CPU): Input, WiFi, Camera, Filesystem, System Logic
- Core 1 (GPU): Graphics, Display, UI, All Rendering
- Task Manager App: Real-time CPU/GPU usage monitoring
- FreeRTOS Task Scheduling with Mutex-Protected LCD Access
- Inter-Core Communication via Command Queues
*/
#include <SPI.h>
#include <Wire.h>
#include <driver/i2s.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <time.h>
#include <HTTPClient.h>
#include <FS.h>
#include <SD_MMC.h>
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ---------- Audio (I2S Codec) Pins ----------
#define PIN_I2S_MCLK  12
#define PIN_I2S_BCLK  13
#define PIN_I2S_LRCK  15
#define PIN_I2S_DOUT  16
#define PIN_I2S_DIN   14
#define I2S_SAMPLE_RATE 16000

// ---------- Pin definitions ----------
#define PIN_LCD_MOSI   1
#define PIN_LCD_MISO   2
#define PIN_LCD_DC     3
#define PIN_LCD_SCLK   5
#define PIN_LCD_BL     6
#define PIN_I2C_SDA    8
#define PIN_I2C_SCL    7
#define LCD_WIDTH      480
#define LCD_HEIGHT     320

// Camera Pins
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  38
#define CAM_PIN_SIOD  8
#define CAM_PIN_SIOC  7
#define CAM_PIN_D7    21
#define CAM_PIN_D6    39
#define CAM_PIN_D5    40
#define CAM_PIN_D4    42
#define CAM_PIN_D3    46
#define CAM_PIN_D2    48
#define CAM_PIN_D1    47
#define CAM_PIN_D0    45
#define CAM_PIN_VSYNC 17
#define CAM_PIN_HREF  18
#define CAM_PIN_PCLK  41

// TCA9554 IO-expander
#define TCA9554_ADDR         0x20
#define TCA9554_REG_OUTPUT   0x01
#define TCA9554_REG_CONFIG   0x03
#define TCA9554_CAM_PWDN_BIT 0
#define TCA9554_PA_CTRL_BIT  7
#define TCA9554_LCD_RST_BIT  1
#define TCA9554_SD_CS_BIT    3

// ---------- SD Card (SDMMC 1-bit) Pins ----------
#define SD_MMC_CLK_PIN  11
#define SD_MMC_CMD_PIN  10
#define SD_MMC_D0_PIN   9

// ES8311 Audio Codec
#define ES8311_ADDR 0x18

// FT6336 Touch
uint8_t FT6336_ADDR = 0x38;
const uint8_t FT6336_CANDIDATE_ADDRS[] = {0x38, 0x15, 0x2A, 0x5D};
#define FT6336_REG_TD_STATUS 0x02
#define FT6336_REG_P1_XH     0x03
#define FT6336_REG_P1_XL     0x04
#define FT6336_REG_P1_YH     0x05
#define FT6336_REG_P1_YL     0x06

// IMU (Auto-detect LSM6DS3 @ 0x6A or MPU6050/ICM42670 @ 0x68)
uint8_t imuAddr = 0;
bool imuAvailable = false;
int16_t accelX = 0;

SPIClass lcdSPI(FSPI);
Preferences prefs;

// ---------- Multi-Core Task Management ----------
TaskHandle_t gpuTaskHandle = nullptr;
TaskHandle_t cpuTaskHandle = nullptr;

// GPU Task Commands (Core 1 - Graphics Only)
enum GpuCommand {
  GPU_CMD_NONE,
  GPU_CMD_FILL_RECT,
  GPU_CMD_DRAW_TEXT,
  GPU_CMD_DRAW_BITMAP,
  GPU_CMD_FILL_SCREEN,
  GPU_CMD_UPDATE_DISPLAY,
  GPU_CMD_DRAW_LAUNCHER,
  GPU_CMD_DRAW_CAMERA,
  GPU_CMD_DRAW_GAME,
  GPU_CMD_CUSTOM_DRAW
};

struct GpuTaskMessage {
  GpuCommand cmd;
  int params[8];          // Generic parameters (x, y, w, h, color, etc.)
  const char* text;       // For text commands
  const uint16_t* bitmap; // For bitmap commands
  void (*customFunc)();   // For custom draw functions
};

QueueHandle_t gpuQueue = nullptr;
SemaphoreHandle_t lcdMutex = nullptr;

// CPU Core (Core 0) Statistics
volatile unsigned long cpuTaskCycles = 0;
volatile unsigned long cpuIdleTime = 0;
volatile unsigned long gpuTaskCycles = 0;
volatile unsigned long gpuIdleTime = 0;
unsigned long lastCpuStatsUpdate = 0;

// Task Manager State
int taskManagerTab = 0; // 0=Overview, 1=Performance, 2=Tasks
#define TM_TAB_COUNT 3
const char* tmTabNames[] = {"OVERVIEW", "PERFORMANCE", "TASKS"};

// Live refresh: data updates every 250ms, but only the pixels that actually
// changed since the last refresh get redrawn (bars deltas, changed digits,
// one new graph column) instead of a full-screen redraw every tick.
#define TM_REFRESH_INTERVAL_MS 250
unsigned long tmLastRefresh = 0;
int tmLastCpuUsage = -1, tmLastGpuUsage = -1;   // -1 = force full draw next time
int tmLastCpuBarW = -1, tmLastGpuBarW = -1;
int tmLastGraphCol = -1;                        // last drawn graph column, for scroll-draw

// Floating overlay: a small always-on-top HUD (independent of which tab/app
// is open) showing live CPU/GPU % once enabled from Task Manager. Position
// is draggable, so X/Y are mutable state rather than fixed defines.
bool tmOverlayEnabled = false;
#define TM_OVERLAY_W 90
#define TM_OVERLAY_H 24
int tmOverlayX = LCD_WIDTH - TM_OVERLAY_W;   // current top-left position
int tmOverlayY = 0;
int tmOverlayLastCpu = -1, tmOverlayLastGpu = -1;
int tmOverlayLastX = -9999, tmOverlayLastY = -9999; // last position actually drawn on screen
bool tmOverlayDragging = false;
int tmOverlayDragOffX = 0, tmOverlayDragOffY = 0;   // touch point offset from overlay's top-left
unsigned long tmOverlayLastDragRedraw = 0;
#define TM_OVERLAY_DRAG_REDRAW_MS 60

// Performance history for graphs
#define HISTORY_SIZE 60
int cpuHistory[HISTORY_SIZE];
int gpuHistory[HISTORY_SIZE];
int historyIndex = 0;

// Forward declarations for multi-core functions
void gpuTask(void* pvParameters);
void cpuTask(void* pvParameters);
void processTouchInput(uint16_t tx, uint16_t ty, bool pressEdge, bool touched);
void processHoldLogic(uint16_t tx, uint16_t ty, bool touched);
void updateScreenSaverLogic();
void processCameraFrame();
void drawTaskManager();
void handleTaskManagerTouch(uint16_t tx, uint16_t ty);
void drawTaskManagerOverview();
void drawTaskManagerPerformance();
void drawTaskManagerTasks();
void tmRefreshTick();
void drawTmOverlay();
void eraseTmOverlay();
void redrawUnderlyingModeThenOverlay();
void tmPartialUpdateOverviewBars(int cpuUsage, int gpuUsage);
void tmPartialUpdateGraphColumn(int col, int cpuUsage, int gpuUsage);
void ipScanStep();
void ipScanFinish(bool cancelled);
void drawWShop();
void handleWShopTouch(uint16_t tx, uint16_t ty);
void openWShop();
bool wshopFetchCppFileList();

// PSRAM framebuffer mirroring screen content (declared here so all functions
// below can see it; it is allocated later, near setup())
uint16_t *lcdFB = nullptr;

// ---------- Colors (RGB565) ----------
#define COL_BLACK   0x0000
#define COL_WHITE   0xFFFF
#define COL_RED     0xF800
#define COL_GREEN   0x07E0
#define COL_BLUE    0x001F
#define COL_YELLOW  0xFFE0
#define COL_GRAY    0xC618
#define COL_DKGRAY  0x8410
#define COL_TITLEBAR 0x10A2
#define COL_MINT    0x07EA
#define COL_MINT_DK 0x05C4
#define COL_MINT_LT 0x1FF2

// ---------- App modes ----------
enum AppMode {
  MODE_SPLASH, MODE_LAUNCHER, MODE_TICTACTOE, MODE_TICTACTOE_AI,
  MODE_SKETCHPAD, MODE_CAMERA, MODE_CAMERA_DIALOG, MODE_MSGBOX,
  MODE_SETTINGS, MODE_SETTINGS_PRIVACY, MODE_SETTINGS_PREFS, MODE_SETTINGS_SOUND, MODE_SETTINGS_TIME,
  MODE_WIFI, MODE_WIFI_PASSWORD, MODE_PIN_ENTRY, MODE_NOTEPAD,
  MODE_CHECKERS, MODE_CHECKERS_AI, MODE_MINESWEEPER, MODE_POWER_MENU,
  MODE_RACING, MODE_RACING_DIFFICULTY, MODE_GHOSTING_WARNING, MODE_GHOSTING, MODE_CLOCK,
  MODE_START_MENU, MODE_SCREENSAVER, MODE_IP_EXPLORER, MODE_IP_VIEWER,
  MODE_SPACE_FIGHTERS, MODE_SF_DIFFICULTY, MODE_SETTINGS_SAVER,
  MODE_DRIVER, MODE_FILE_BROWSER, MODE_FILENAME_INPUT, MODE_SD_CONFIRM,
  MODE_GMAIL_SIGNIN, MODE_EMAIL_INBOX, MODE_EMAIL_COMPOSE, MODE_TASK_MANAGER, MODE_WSHOP
};

AppMode currentMode = MODE_SPLASH;
AppMode msgBoxReturnMode = MODE_LAUNCHER;

enum CameraMode { CAM_MODE_NONE, CAM_MODE_LIVE, CAM_MODE_DIAGNOSTIC };
CameraMode cameraMode = CAM_MODE_NONE;
int cameraRotation = 90;
int camPhotoCount = 0;
int camLastDispW = 0, camLastDispH = 0;
unsigned long camShutterFlashUntil = 0;

// ---------- UI Layout ----------
#define BAR_Y  (LCD_HEIGHT - 40)
#define BAR_H  40
#define BAR_BTN_COUNT 8
#define BAR_BTN_W (LCD_WIDTH / BAR_BTN_COUNT)
// File Browser layout (needed early for refreshFileList)
#define FB_HEADER_H 40
#define FB_ROW_H 22
#define FB_BOTTOM_H 30
#define FB_LIST_Y0 (FB_HEADER_H)
#define FB_LIST_Y1 (BAR_Y - FB_BOTTOM_H)
#define FB_MAX_ROWS ((FB_LIST_Y1 - FB_LIST_Y0) / FB_ROW_H)
#define GRID_COLS 8
#define GRID_ROWS 3
#define CELL_W (LCD_WIDTH / GRID_COLS)
#define CELL_H (LCD_HEIGHT / GRID_ROWS)
#define DLG_W 320
#define DLG_H 180
#define DLG_X ((LCD_WIDTH - DLG_W) / 2)
#define DLG_Y ((LCD_HEIGHT - DLG_H) / 2)
#define BTN_W 90
#define BTN_H 35
#define BTN_SPACING 10
#define MBX_W 300
#define MBX_H 150
#define MBX_X ((LCD_WIDTH - MBX_W) / 2)
#define MBX_Y ((LCD_HEIGHT - MBX_H) / 2)
#define MBX_TITLEBAR_H 22
#define MBX_ICON_CX (MBX_X + 40)
#define MBX_ICON_CY (MBX_Y + MBX_TITLEBAR_H + 42)
#define MBX_ICON_R  20
#define MBX_OK_W 70
#define MBX_OK_H 26
#define MBX_OK_X (MBX_X + (MBX_W - MBX_OK_W)/2)
#define MBX_OK_Y (MBX_Y + MBX_H - MBX_OK_H - 12)

enum MsgBoxType { MSG_CRIT, MSG_WARN, MSG_INFO, MSG_QUESTION };
char msgBoxTitle[24] = "";
char msgBoxLine1[28] = "";
char msgBoxLine2[28] = "";
MsgBoxType msgBoxType = MSG_INFO;
bool msgBoxResetPending = false;
// msgBoxFormatSdPending removed — uses MODE_SD_CONFIRM dialog now

// ---------- Global Settings & State ----------
int systemVolume = 50;
bool settingClickSounds = true;
bool settingCameraAccess = true;
bool settingMicAccess = true;
bool settingTouchLogging = false;
bool settingAutoLock = false;
bool settingStartupJingle = true;
bool settingGameSounds = true;
bool settingScreensaverEnabled = true;
int settingTimezoneOffset = 0;
int settingBrightness = 100;

// Gmail Settings
char gmailUsername[64] = "";
char gmailPassword[64] = "";
bool gmailConfigured = false;
bool firstBootCompleted = false;

// Screen Saver Settings
int settingSSTimeout = 5; // Minutes
int settingSSMode = 0; // 0=Smile, 1=DVD, 2=W
// Screen Saver settings screen is now reachable from more than one place
// (PREFERENCES, and directly from the SETTINGS main menu) - remember which
// one so its BACK button, and goBack(), return to the right place instead
// of always assuming PREFERENCES.
AppMode settingsSaverReturnMode = MODE_SETTINGS_PREFS;

int racingHighScore = 0;
int sfHighScore = 0;
bool sfGameOver = false; // declared early: used in goBack() before the rest of the Space Fighters section
int sfDifficulty = 0; // 0=aEASY, 1=MEDIUM, 2=HARD

enum ThemeIndex { THEME_MINT = 0, THEME_DARK = 1, THEME_BLUE = 2, THEME_RED = 3 };
int settingThemeIndex = THEME_MINT;
const char* THEME_NAMES[4] = {"MINT", "DARK", "BLUE", "RED"};

bool pinEnabled = false;
char pinCode[5] = "1234";
char pinEntry[5] = "";
int pinEntryLen = 0;
bool pinSetupMode = false;
char pinNewCode[5] = "";
int pinNewLen = 0;

#define MAX_WIFI_NETWORKS 15
String wifiSSIDs[MAX_WIFI_NETWORKS];
int wifiRSSIs[MAX_WIFI_NETWORKS];
int wifiNetworkCount = 0;
bool wifiConnected = false;
String wifiConnectedSSID = "";
String wifiPassword = "";
String wifiTargetSSID = "";
int wifiScanState = 0;
int wifiScrollOffset = 0;
bool shiftActive = false;
bool isNumLayout = false;
String notepadText = "";

const char* kbAlpha[4][10] = {
  {"Q","W","E","R","T","Y","U","I","O","P"},
  {"A","S","D","F","G","H","J","K","L","@"},
  {"SHIFT","Z","X","C","V","B","N","M",".","DEL"},
  {"NUM","SPACE","SPACE","SPACE","SPACE","SPACE","SPACE","SPACE","OK","OK"}
};
const char* kbNum[4][10] = {
  {"1","2","3","4","5","6","7","8","9","0"},
  {"@","#","$","%","&","-","+","(",")","DEL"},
  {"!","=","{","}","[","]",";","'",":","DEL"},
  {"ABC","SPACE","SPACE","SPACE","SPACE","SPACE","SPACE","SPACE","OK","OK"}
};

unsigned long splashStartTime = 0;

// Ghosting App State
unsigned long ghostingStartTime = 0;
unsigned long ghostingLastToggle = 0;
bool ghostingState = false;
bool ssAutoTriggered = false;
unsigned long lastActivityTime = 0;

// Start Menu State
int startMenuTab = 2; // 0=Games, 1=System, 2=All, 3=Removed
const char* startAppNames[] = {"TTT", "TTT AI", "CHECKERS", "CHK AI", "MINES", "RACING", "SPACE FTR", "DRIVER", "SETTINGS", "WIFI", "CLOCK", "SKETCH", "NOTEPAD", "CAMERA", "IP EXP", "FILES", "GHOST", "EMAIL", "TASK MAN", "W SHOP"};
int startAppModes[] = {MODE_TICTACTOE, MODE_TICTACTOE_AI, MODE_CHECKERS, MODE_CHECKERS_AI, MODE_MINESWEEPER, MODE_RACING_DIFFICULTY, MODE_SF_DIFFICULTY, MODE_DRIVER, MODE_SETTINGS, MODE_WIFI, MODE_CLOCK, MODE_SKETCHPAD, MODE_NOTEPAD, MODE_CAMERA_DIALOG, MODE_IP_EXPLORER, MODE_FILE_BROWSER, MODE_GHOSTING_WARNING, MODE_EMAIL_INBOX, MODE_TASK_MANAGER, MODE_WSHOP};
int startAppCats[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // 0=Games, 1=System
int startAppCount = 20;
bool startAppRemoved[20] = {false}; // true = app is hidden and lives in the REMOVED tab
int appHoldIdx = -1; // start-menu grid app index currently being pressed, or -1
unsigned long appHoldStart = 0;
bool appHoldFired = false; // true once a long-press has resolved (entered edit mode) for the current touch
// (holding an icon now removes it directly — no separate edit-mode toggle needed)

// Launcher Edit Mode State (mirrors Start Menu's hold-to-remove behavior)
// (holding a tile now removes it directly — no separate edit-mode toggle needed)
int launcherHoldIdx = -1;      // launcher grid tile index currently being pressed, or -1
unsigned long launcherHoldStart = 0;
bool launcherHoldFired = false; // true once a long-press has resolved (entered edit mode) for the current touch
// Maps a launcher tile index (0-17) to its corresponding startApp[] index so hiding an app
// from either screen stays in sync.
const int launcherToStartIdx[20] = {0, 1, 11, 13, 8, 9, 12, 2, 3, 4, 5, 16, 10, 14, 6, 7, 15, 17, 18, 19};

// IP Explorer State
String ipList[254];
int ipCount = 0;
bool ipScanning = false;
String ipViewerContent = "";
String ipViewerUrl = "";
int ipScanProgress = 0; // 0-254 for progress bar
// Background/non-blocking scan state. The scan now steps ONE address per
// pass of cpuTask's loop instead of blocking Core 0 in a tight for-loop.
// Only the actual network probe (ipScanStep's HTTPClient call) is "the
// scanning" — every other function (UI drawing, touch input, other apps)
// keeps running normally via the GPU queue, so the user can back out of
// IP Explorer and use the rest of the OS while a scan runs in the background.
int ipScanIndex = 1;                 // next octet to probe (1-254)
IPAddress ipScanBaseIP;              // captured at scan start
bool ipScanCancelled = false;        // set by explicit STOP, power press, or wifi drop
unsigned long ipScanLastStepTime = 0;
#define IP_SCAN_STEP_INTERVAL_MS 15  // throttle between probes so Core 0 stays responsive

// ---------- Screenshot / Boot Button State ----------
#define BOOT_PIN 0
unsigned long bootPressStart = 0;
bool bootHeld = false;
int screenshotCount = 0;

// Screenshot animation
#define SS_THUMB_W 64
#define SS_THUMB_H 44
uint16_t *ssThumb = nullptr;   // downscaled thumbnail
uint16_t *ssBg = nullptr;       // saved background under thumbnail
unsigned long ssAnimStart = 0;
bool ssAnimActive = false;
#define SS_ANIM_MS 1200

// ---------- File Browser State ----------
#define MAX_FILES 64
String fileList[MAX_FILES];
String fileSizes[MAX_FILES];
int fileCount = 0;
int fileScrollOffset = 0;
String fileToOpen = "";
bool fileViewerActive = false; // True when an image is being viewed in file browser
String fileViewerPath = ""; // path of the .espimg currently shown in the viewer
bool sdCardMounted = false;
int fileMenuIndex = -1; // -1 = no menu, >=0 = context menu for that file index
String renameTarget = ""; // full path of file being renamed

// ---------- Filename Input State ----------
String filenameInput = "";
int saveContext = 0; // 0=notepad, 1=sketchpad
AppMode saveReturnMode = MODE_LAUNCHER;

// ---------- Sketchpad Framebuffer (PSRAM) ----------
uint16_t *sketchFB = nullptr;
#define SKETCH_FB_W LCD_WIDTH
#define SKETCH_FB_H (BAR_Y - SKETCH_TOOLBAR_H)

// ---------- DRIVER Racing Game State ----------
#define DRV_MAX_NPCS 3
struct DrvNpc { int x, y, speed; bool active; };
DrvNpc drvNpcs[DRV_MAX_NPCS];
int drvCarX = 0;
int drvScore = 0;
int drvSpeed = 3;
bool drvGameOver = false;
int drvRoadOffset = 0;
unsigned long drvLastUpdate = 0;
int drvSteer = 0;
int drvLap = 0;
int drvHighScore = 0;

// =====================================================================
// ---------- PERSISTENT SETTINGS (NVS) ----------
// =====================================================================
void saveSettings() {
  prefs.begin("wave_os", false);
  prefs.putInt("volume", systemVolume);
  prefs.putBool("click", settingClickSounds);
  prefs.putBool("cam", settingCameraAccess);
  prefs.putBool("mic", settingMicAccess);
  prefs.putBool("touch", settingTouchLogging);
  prefs.putBool("lock", settingAutoLock);
  prefs.putBool("jingle", settingStartupJingle);
  prefs.putBool("gamesnd", settingGameSounds);
  prefs.putBool("ssaver", settingScreensaverEnabled);
  prefs.putInt("tz", settingTimezoneOffset);
  prefs.putInt("theme", settingThemeIndex);
  prefs.putInt("bright", settingBrightness);
  prefs.putBool("pin_en", pinEnabled);
  prefs.putString("pin", pinCode);
  prefs.putString("wifi_ssid", wifiTargetSSID);
  prefs.putString("wifi_pass", wifiPassword);
  prefs.putInt("cam_rot", cameraRotation);
  
  // New SS Settings
  prefs.putInt("ss_timeout", settingSSTimeout);
  prefs.putInt("ss_mode", settingSSMode);

  // Gmail Settings
  prefs.putString("gmail_user", gmailUsername);
  prefs.putString("gmail_pass", gmailPassword);
  prefs.putBool("gmail_config", gmailConfigured);
  prefs.putBool("first_boot", firstBootCompleted);

  uint32_t removedMask = 0;
  for (int i = 0; i < startAppCount && i < 32; i++) if (startAppRemoved[i]) removedMask |= ((uint32_t)1 << i);
  prefs.putUInt("app_hide", removedMask);

  prefs.end();
}

void saveHighScores() {
  prefs.begin("wave_os", false);
  prefs.putInt("race_hi", racingHighScore);
  prefs.putInt("sf_hi", sfHighScore);
  prefs.putInt("drv_hi", drvHighScore);
  prefs.end();
}

// ---------- SD Card File Helpers ----------
bool saveEspTxt(const char* filename, const String& text) {
  if (!sdCardMounted) { Serial.println("SD not mounted"); return false; }
  File f = SD_MMC.open(filename, "w");
  if (!f) { Serial.println("Failed to save file"); return false; }
  f.write((const uint8_t*)text.c_str(), text.length());
  f.close();
  Serial.printf("Saved %s (%d bytes)\n", filename, text.length());
  return true;
}

bool loadEspTxt(const char* filename, String& outText) {
  File f = SD_MMC.open(filename, "r");
  if (!f) return false;
  outText = f.readString();
  f.close();
  return true;
}

bool saveEspImg(const char* filename, uint16_t x0, uint16_t y0, uint16_t w, uint16_t h) {
  if (!sdCardMounted) { Serial.println("SD not mounted"); return false; }
  uint16_t *buf = (uint16_t *)heap_caps_malloc((size_t)w * h * 2, MALLOC_CAP_SPIRAM);
  if (!buf) { Serial.println("Screenshot buffer alloc failed"); return false; }
  // Read from PSRAM framebuffer (LCD MISO not connected)
  if (lcdFB) {
    for (uint16_t y = 0; y < h; y++)
      for (uint16_t x = 0; x < w; x++) {
        uint16_t sx = x0 + x, sy = y0 + y;
        if (sx < LCD_WIDTH && sy < LCD_HEIGHT)
          buf[y * w + x] = lcdFB[sy * LCD_WIDTH + sx];
        else
          buf[y * w + x] = 0;
      }
  } else { memset(buf, 0, (size_t)w * h * 2); }
  File f = SD_MMC.open(filename, "w");
  if (!f) { Serial.println("Failed to open img file for write"); free(buf); return false; }
  // Simple header: width(2) height(2) reserved(2)
  f.write((uint8_t*)&w, 2);
  f.write((uint8_t*)&h, 2);
  uint16_t zero = 0; f.write((uint8_t*)&zero, 2);
  // Write in chunks to avoid WDT
  size_t total = (size_t)w * h * 2;
  size_t written = 0;
  const uint8_t *p = (const uint8_t*)buf;
  while (written < total) {
    size_t chunk = min((size_t)4096, total - written);
    f.write(p + written, chunk);
    written += chunk;
    if (written % 16384 == 0) yield();
  }
  f.close();
  free(buf);
  Serial.printf("Saved image %s (%dx%d)\n", filename, w, h);
  return true;
}

// Save image from an existing buffer (no GRAM read needed — for sketchpad)
bool saveEspImgFromBuffer(const char* filename, uint16_t *buf, uint16_t w, uint16_t h) {
  if (!sdCardMounted) { Serial.println("SD not mounted"); return false; }
  File f = SD_MMC.open(filename, "w");
  if (!f) { Serial.println("Failed to open img file for write"); return false; }
  f.write((uint8_t*)&w, 2);
  f.write((uint8_t*)&h, 2);
  uint16_t zero = 0; f.write((uint8_t*)&zero, 2);
  size_t total = (size_t)w * h * 2;
  size_t written = 0;
  const uint8_t *p = (const uint8_t*)buf;
  while (written < total) {
    size_t chunk = min((size_t)4096, total - written);
    f.write(p + written, chunk);
    written += chunk;
    if (written % 16384 == 0) yield();
  }
  f.close();
  Serial.printf("Saved image %s (%dx%d) from buffer\n", filename, w, h);
  return true;
}

bool loadEspImg(const char* filename, uint16_t &outW, uint16_t &outH, uint16_t *&outBuf) {
  File f = SD_MMC.open(filename, "r");
  if (!f) return false;
  f.read((uint8_t*)&outW, 2);
  f.read((uint8_t*)&outH, 2);
  uint16_t reserved; f.read((uint8_t*)&reserved, 2);
  size_t pixCount = (size_t)outW * outH;
  outBuf = (uint16_t *)heap_caps_malloc(pixCount * 2, MALLOC_CAP_SPIRAM);
  if (!outBuf) { f.close(); return false; }
  f.read((uint8_t*)outBuf, pixCount * 2);
  f.close();
  return true;
}

void takeScreenshot() {
  // 1. Allocate buffer for screen capture (copy from framebuffer)
  uint16_t *buf = (uint16_t *)heap_caps_malloc((size_t)LCD_WIDTH * LCD_HEIGHT * 2, MALLOC_CAP_SPIRAM);
  if (!buf) { playError(); Serial.println("SS buf alloc failed"); char psramMsg[28]; snprintf(psramMsg, sizeof(psramMsg), "PSRAM SIZE: %d", ESP.getPsramSize()); showMessageBox(MSG_CRIT, "SCREENSHOT FAILED", "NO PSRAM BUFFER", psramMsg, currentMode); return; }

  // 2. Copy from PSRAM framebuffer (no GRAM read needed — MISO not connected)
  if (lcdFB) {
    memcpy(buf, lcdFB, (size_t)LCD_WIDTH * LCD_HEIGHT * 2);
  } else {
    memset(buf, 0, (size_t)LCD_WIDTH * LCD_HEIGHT * 2);
  }

  // 3. Quick backlight flash (visual feedback)
  int savedBL = settingBrightness * 255 / 100;
  analogWrite(PIN_LCD_BL, 0); delay(40); analogWrite(PIN_LCD_BL, savedBL);

  // 4. Save to SD card
  if (sdCardMounted) {
    prefs.begin("wave_os", false);
    screenshotCount = prefs.getInt("ss_cnt", 0);
    char fname[32]; snprintf(fname, sizeof(fname), "/shot%d.espimg", screenshotCount);
    prefs.putInt("ss_cnt", screenshotCount + 1);
    prefs.end();
    File f = SD_MMC.open(fname, "w");
    if (f) {
      uint16_t w = LCD_WIDTH, h = LCD_HEIGHT;
      f.write((uint8_t*)&w, 2);
      f.write((uint8_t*)&h, 2);
      uint16_t zero = 0; f.write((uint8_t*)&zero, 2);
      size_t total = (size_t)w * h * 2;
      const uint8_t *p = (const uint8_t*)buf;
      size_t written = 0;
      while (written < total) {
        size_t chunk = min((size_t)4096, total - written);
        f.write(p + written, chunk);
        written += chunk;
        if (written % 16384 == 0) yield();
      }
      f.close();
      Serial.printf("Screenshot saved: %s\n", fname);
    } else {
      Serial.println("Failed to save screenshot");
      showMessageBox(MSG_WARN, "SCREENSHOT FAILED", "COULD NOT OPEN", "FILE ON SD CARD", currentMode);
    }
  } else {
    Serial.println("SD not mounted, screenshot not saved");
    showMessageBox(MSG_WARN, "SCREENSHOT FAILED", "SD CARD NOT", "MOUNTED", currentMode);
  }

  // 5. Create downscaled thumbnail for animation
  if (!ssThumb) ssThumb = (uint16_t*)heap_caps_malloc(SS_THUMB_W * SS_THUMB_H * 2, MALLOC_CAP_SPIRAM);
  if (!ssBg)   ssBg   = (uint16_t*)heap_caps_malloc((SS_THUMB_W + 4) * (SS_THUMB_H + 4) * 2, MALLOC_CAP_SPIRAM);
  if (ssThumb && buf) {
    for (int y = 0; y < SS_THUMB_H; y++)
      for (int x = 0; x < SS_THUMB_W; x++)
        ssThumb[y * SS_THUMB_W + x] = buf[y * LCD_HEIGHT / SS_THUMB_H * LCD_WIDTH + x * LCD_WIDTH / SS_THUMB_W];
  }

  free(buf);

  // 6. Start animation
  ssAnimActive = true;
  ssAnimStart = millis();
  playSuccess();
}

void drawScreenshotAnim() {
  if (!ssAnimActive || !ssThumb || !ssBg) return;
  unsigned long elapsed = millis() - ssAnimStart;
  int bw = SS_THUMB_W + 4, bh = SS_THUMB_H + 4;
  int baseBx = LCD_WIDTH - SS_THUMB_W - 6;
  int baseBy = BAR_Y - SS_THUMB_H - 6;

  if (elapsed >= SS_ANIM_MS) {
    // Animation done — restore background from ssBg to LCD and framebuffer
    int bx = baseBx, by = baseBy;
    // Restore framebuffer
    if (lcdFB) {
      for (int yy = 0; yy < bh; yy++)
        for (int xx = 0; xx < bw; xx++) {
          int sx = bx + xx, sy = by + yy;
          if (sx >= 0 && sx < LCD_WIDTH && sy >= 0 && sy < LCD_HEIGHT)
            lcdFB[sy * LCD_WIDTH + sx] = ssBg[yy * bw + xx];
        }
    }
    // Restore LCD
    lcdSetAddrWindow(bx, by, bx + bw - 1, by + bh - 1);
    lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_LCD_DC, HIGH);
    static uint8_t rstBuf[(SS_THUMB_W + 4) * 2];
    for (int yy = 0; yy < bh; yy++) {
      for (int xx = 0; xx < bw; xx++) {
        uint16_t c = ssBg[yy * bw + xx];
        rstBuf[xx * 2] = c >> 8; rstBuf[xx * 2 + 1] = c & 0xFF;
      }
      lcdSPI.transferBytes(rstBuf, nullptr, bw * 2);
    }
    lcdSPI.endTransaction();
    ssAnimActive = false;
    return;
  }

  // Calculate X offset for slide animation
  int offsetX = 0;
  if (elapsed < 200) {
    offsetX = (SS_THUMB_W + 10) * (1 - (int)elapsed / 200);
  } else if (elapsed > 1000) {
    offsetX = (SS_THUMB_W + 10) * ((int)(elapsed - 1000) / 200);
  }

  int bx = baseBx + offsetX;
  int by = baseBy;

  if (bx >= LCD_WIDTH || bx + bw <= 0) return;

  // Save background from framebuffer (instant — no SPI read)
  if (lcdFB) {
    for (int yy = 0; yy < bh; yy++)
      for (int xx = 0; xx < bw; xx++) {
        int sx = bx + xx, sy = by + yy;
        if (sx >= 0 && sx < LCD_WIDTH && sy >= 0 && sy < LCD_HEIGHT)
          ssBg[yy * bw + xx] = lcdFB[sy * LCD_WIDTH + sx];
        else
          ssBg[yy * bw + xx] = 0;
      }
  } else {
    memset(ssBg, 0, bw * bh * 2);
  }

  // Draw white border on LCD and framebuffer
  lcdFillRect(bx, by, bw, bh, COL_WHITE);

  // Draw thumbnail on LCD
  lcdSetAddrWindow(bx + 2, by + 2, bx + 2 + SS_THUMB_W - 1, by + 2 + SS_THUMB_H - 1);
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_DC, HIGH);
  static uint8_t tBuf[SS_THUMB_W * 2];
  for (int y = 0; y < SS_THUMB_H; y++) {
    for (int x = 0; x < SS_THUMB_W; x++) {
      uint16_t c = ssThumb[y * SS_THUMB_W + x];
      tBuf[x * 2] = c >> 8;
      tBuf[x * 2 + 1] = c & 0xFF;
    }
    lcdSPI.transferBytes(tBuf, nullptr, SS_THUMB_W * 2);
  }
  lcdSPI.endTransaction();

  // Draw thumbnail to framebuffer
  if (lcdFB) {
    for (int y = 0; y < SS_THUMB_H; y++)
      for (int x = 0; x < SS_THUMB_W; x++) {
        int sx = bx + 2 + x, sy = by + 2 + y;
        if (sx >= 0 && sx < LCD_WIDTH && sy >= 0 && sy < LCD_HEIGHT)
          lcdFB[sy * LCD_WIDTH + sx] = ssThumb[y * SS_THUMB_W + x];
      }
  }

  // Draw "SAVED!" text (lcdDrawText updates framebuffer via lcdFillRect)
  lcdDrawText(bx + 2, by + bh + 2, "SAVED!", COL_WHITE, 1);
}

String formatFileSize(size_t bytes) {
  char buf[16];
  if (bytes < 1024) snprintf(buf, sizeof(buf), "%dB", (int)bytes);
  else if (bytes < 1048576) snprintf(buf, sizeof(buf), "%dKB", (int)(bytes / 1024));
  else snprintf(buf, sizeof(buf), "%dMB", (int)(bytes / 1048576));
  return String(buf);
}

void refreshFileList() {
  fileCount = 0;
  // Don't reset fileScrollOffset here — clamp after building list
  if (!sdCardMounted) return;
  File root = SD_MMC.open("/");
  if (!root) return;
  File entry = root.openNextFile();
  while (entry && fileCount < MAX_FILES) {
    String name = entry.name();
    // Normalize: strip leading slash if present
    if (name.startsWith("/")) name = name.substring(1);
    // Skip hidden files and system files
    if (name.length() > 0 && name[0] != '.') {
      fileList[fileCount] = name;
      fileSizes[fileCount] = entry.isDirectory() ? "<DIR>" : formatFileSize(entry.size());
      fileCount++;
    }
    entry = root.openNextFile();
  }
  root.close();
  // Clamp scroll offset
  int maxOffset = max(0, fileCount - FB_MAX_ROWS);
  if (fileScrollOffset > maxOffset) fileScrollOffset = maxOffset;
}

void initSDCard() {
  // Configure EXIO3 (SD_CS) — set HIGH first, then make it an output
  tca9554SetBit(TCA9554_SD_CS_BIT, true); // HIGH before output config
  uint8_t cfg = tca9554ReadReg(TCA9554_REG_CONFIG);
  cfg &= ~(1 << TCA9554_SD_CS_BIT); // set as output
  tca9554WriteReg(TCA9554_REG_CONFIG, cfg);
  tca9554SetBit(TCA9554_SD_CS_BIT, true); // ensure HIGH
  delay(100);
  // Initialize SD_MMC in 1-bit mode
  if (!SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN)) {
    Serial.println("SD_MMC setPins failed");
    sdCardMounted = false;
    return;
  }
  sdCardMounted = SD_MMC.begin("/sdcard", true, false, 20000);
  if (sdCardMounted) {
    Serial.printf("SD card mounted: %lluMB / %lluMB used\n", SD_MMC.totalBytes() / 1048576, SD_MMC.usedBytes() / 1048576);
  } else {
    Serial.println("SD card mount failed");
  }
}

void formatSdCard() {
  if (!sdCardMounted) return;
  // Delete all files in root
  File root = SD_MMC.open("/");
  if (!root) return;
  File entry = root.openNextFile();
  while (entry) {
    String name = entry.name();
    entry.close();
    String path = "/" + name;
    SD_MMC.remove(path.c_str());
    entry = root.openNextFile();
  }
  root.close();
  refreshFileList();
}

void loadSettings() {
  prefs.begin("wave_os", true);
  systemVolume = min(90, (int)prefs.getInt("volume", 50));
  settingClickSounds = prefs.getBool("click", true);
  settingCameraAccess = prefs.getBool("cam", true);
  settingMicAccess = prefs.getBool("mic", true);
  settingTouchLogging = prefs.getBool("touch", false);
  settingAutoLock = prefs.getBool("lock", false);
  settingStartupJingle = prefs.getBool("jingle", true);
  settingGameSounds = prefs.getBool("gamesnd", true);
  settingScreensaverEnabled = prefs.getBool("ssaver", true);
  settingTimezoneOffset = prefs.getInt("tz", 0);
  settingThemeIndex = prefs.getInt("theme", THEME_MINT);
  settingBrightness = prefs.getInt("bright", 100);
  
  // New SS Settings
  settingSSTimeout = prefs.getInt("ss_timeout", 5);
  settingSSMode = prefs.getInt("ss_mode", 0);

  // Gmail Settings
  prefs.getString("gmail_user", gmailUsername, sizeof(gmailUsername));
  prefs.getString("gmail_pass", gmailPassword, sizeof(gmailPassword));
  gmailConfigured = prefs.getBool("gmail_config", false);
  firstBootCompleted = prefs.getBool("first_boot", false);

  uint32_t removedMask = prefs.getUInt("app_hide", 0);
  for (int i = 0; i < startAppCount && i < 32; i++) startAppRemoved[i] = (removedMask & ((uint32_t)1 << i)) != 0;
  
  racingHighScore = prefs.getInt("race_hi", 0);
  sfHighScore = prefs.getInt("sf_hi", 0);
  drvHighScore = prefs.getInt("drv_hi", 0);
  
  pinEnabled = prefs.getBool("pin_en", false);
  String savedPin = prefs.getString("pin", "1234");
  strncpy(pinCode, savedPin.c_str(), 4);
  pinCode[4] = '\0';
  wifiTargetSSID = prefs.getString("wifi_ssid", "");
  wifiPassword = prefs.getString("wifi_pass", "");
  cameraRotation = prefs.getInt("cam_rot", 90);
  prefs.end();
}

// ---------- TCA9554 helpers ----------
void tca9554WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCA9554_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t tca9554ReadReg(uint8_t reg) {
  Wire.beginTransmission(TCA9554_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((int)TCA9554_ADDR, 1); return Wire.available() ? Wire.read() : 0xFF;
}
void tca9554SetBit(uint8_t bit, bool state) {
  uint8_t val = tca9554ReadReg(TCA9554_REG_OUTPUT);
  if (state) val |= (1 << bit); else val &= ~(1 << bit);
  tca9554WriteReg(TCA9554_REG_OUTPUT, val);
}
void lcdHardwareReset() {
  uint8_t cfg = tca9554ReadReg(TCA9554_REG_CONFIG); cfg &= ~(1 << TCA9554_LCD_RST_BIT);
  tca9554WriteReg(TCA9554_REG_CONFIG, cfg);
  tca9554SetBit(TCA9554_LCD_RST_BIT, true); delay(20);
  tca9554SetBit(TCA9554_LCD_RST_BIT, false); delay(20);
  tca9554SetBit(TCA9554_LCD_RST_BIT, true); delay(150);
}
void camSetPowerDown(bool down) {
  uint8_t cfg = tca9554ReadReg(TCA9554_REG_CONFIG); cfg &= ~(1 << TCA9554_CAM_PWDN_BIT);
  tca9554WriteReg(TCA9554_REG_CONFIG, cfg); tca9554SetBit(TCA9554_CAM_PWDN_BIT, down);
}
void paEnable(bool en) {
  uint8_t cfg = tca9554ReadReg(TCA9554_REG_CONFIG); cfg &= ~(1 << TCA9554_PA_CTRL_BIT);
  tca9554WriteReg(TCA9554_REG_CONFIG, cfg); tca9554SetBit(TCA9554_PA_CTRL_BIT, en);
}

// ---------- Sound (I2S) ----------
bool soundReady = false;
bool startupJinglePending = true;
void playDesktopJingleOnce() {
  if (startupJinglePending) { startupJinglePending = false; playStartupJingle(); }
}
void es8311WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t es8311ReadReg(uint8_t reg) {
  Wire.beginTransmission(ES8311_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)ES8311_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}
// Maps 0-100 UI volume to the ES8311 DAC volume register (0x32), which is
// a linear 0-255 scale: reg = (vol*256/100)-1, 0 = mute (Espressif's own formula).
void es8311SetVolume(uint8_t volPercent) {
  if (volPercent > 100) volPercent = 100;
  uint8_t reg = (volPercent == 0) ? 0 : (uint8_t)(((uint32_t)volPercent * 256 / 100) - 1);
  es8311WriteReg(0x32, reg);
}

bool es8311Init() {
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ES8311 NOT FOUND on I2C bus");
    return false;
  }
  // ---- Reset ----
  es8311WriteReg(0x00, 0x1F); delay(20);
  es8311WriteReg(0x00, 0x00); delay(20);

  // ---- Clock manager: MCLK arrives on the dedicated MCLK pin (bit7=0) ----
  es8311WriteReg(0x01, 0x3F);

  // ---- Clock coefficients for MCLK=4.096MHz (256 * 16kHz) @ Fs=16kHz,
  // taken from Espressif's ES8311 coefficient table (mclk=4096000/rate=16000 row) ----
  es8311WriteReg(0x02, 0x00);
  es8311WriteReg(0x03, 0x10);
  es8311WriteReg(0x04, 0x10);
  es8311WriteReg(0x05, 0x00);
  es8311WriteReg(0x06, 0x03);
  es8311WriteReg(0x07, 0x00);
  es8311WriteReg(0x08, 0xFF);

  // ---- Serial data port: 16-bit I2S, slave mode ----
  es8311WriteReg(0x09, 0x0C);
  es8311WriteReg(0x0A, 0x0C);

  // ---- Bring the codec out of standby ----
  es8311WriteReg(0x00, 0x80); delay(10);

  // ---- Analog power-up sequence. All of these are "NOT default" registers
  // per Espressif's own driver - reset leaves them powered down/bypassed off,
  // and without these the DAC output never reaches the pins even though I2C
  // and I2S both report healthy. ----
  es8311WriteReg(0x0D, 0x01);   // Power up analog circuitry
  es8311WriteReg(0x0E, 0x02);   // Enable analog PGA / ADC modulator
  es8311WriteReg(0x12, 0x00);   // Power up the DAC itself
  es8311WriteReg(0x13, 0x10);   // Enable DAC output to the HP/speaker driver
  es8311WriteReg(0x1C, 0x6A);   // ADC equalizer bypass, cancel digital DC offset
  es8311WriteReg(0x37, 0x08);   // DAC equalizer bypass

  es8311WriteReg(0x31, 0x00);   // DAC unmute

  es8311SetVolume(systemVolume);

  uint8_t chipId1 = es8311ReadReg(0xFD);
  uint8_t chipId2 = es8311ReadReg(0xFE);
  Serial.printf("ES8311 codec initialized (ID: 0x%02X 0x%02X)\n", chipId1, chipId2);
  return true;
}
void soundInit() {
  bool codecOk = es8311Init();
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8, .dma_buf_len = 512, .use_apll = true, .tx_desc_auto_clear = true, .fixed_mclk = I2S_SAMPLE_RATE * 256
  };
  i2s_pin_config_t pins = { .mck_io_num = PIN_I2S_MCLK, .bck_io_num = PIN_I2S_BCLK, .ws_io_num = PIN_I2S_LRCK, .data_out_num = PIN_I2S_DOUT, .data_in_num = I2S_PIN_NO_CHANGE };
  if (codecOk && i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) == ESP_OK && i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK) {
    i2s_zero_dma_buffer(I2S_NUM_0);
    soundReady = true; paEnable(true); es8311SetVolume(systemVolume); Serial.println("I2S Sound OK");
  } else { Serial.println("I2S/codec init failed"); soundReady = false; }
}
// Call whenever systemVolume changes so the hardware gain and the settings
// screen (and any other UI) always agree with what the codec is doing.
void applyVolume() {
  if (soundReady) es8311SetVolume(systemVolume);
}
void playChord(const float* freqs, int numFreqs, uint16_t durationMs) {
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase[4] = {0, 0, 0, 0}; float phaseInc[4] = {0, 0, 0, 0};
  int nf = numFreqs > 4 ? 4 : numFreqs;
  for (int f = 0; f < nf; f++) phaseInc[f] = 2.0f * PI * freqs[f] / I2S_SAMPLE_RATE;
  float amp = (systemVolume / 100.0f) * (11000.0f / nf); size_t written; uint32_t sent = 0;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      float t = (float)(sent + i) / totalSamples;
      float env = sinf(PI * t); // smooth swell in, then out - ambient pad feel
      float mix = 0;
      for (int f = 0; f < nf; f++) { mix += sinf(phase[f]); phase[f] += phaseInc[f]; if (phase[f] > 2.0f * PI) phase[f] -= 2.0f * PI; }
      int16_t s = (int16_t)(mix * amp * env); buf[i*2] = s; buf[i*2+1] = s;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playBellNote(float freq, uint16_t durationMs, float ampScale) {
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase1 = 0, phase2 = 0, phase3 = 0;
  float inc1 = 2.0f * PI * freq / I2S_SAMPLE_RATE;
  float inc2 = 2.0f * PI * freq * 2.0f / I2S_SAMPLE_RATE; // clean octave (no detune -> no warble)
  float inc3 = 2.0f * PI * freq * 4.0f / I2S_SAMPLE_RATE; // clean 2-octave sparkle, low mix
  float amp = (systemVolume / 100.0f) * 13000.0f * ampScale; size_t written; uint32_t sent = 0;
  const float attackFrac = 0.04f; // short fade-in so the note starts soft, not clicky
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      float t = (float)(sent + i) / totalSamples;
      float env;
      if (t < attackFrac) env = t / attackFrac;
      else { float t2 = (t - attackFrac) / (1.0f - attackFrac); env = expf(-2.4f * t2); }
      float mix = sinf(phase1) * 0.68f + sinf(phase2) * 0.22f + sinf(phase3) * 0.10f;
      int16_t s = (int16_t)(mix * amp * env); buf[i*2] = s; buf[i*2+1] = s;
      phase1 += inc1; if (phase1 > 2.0f * PI) phase1 -= 2.0f * PI;
      phase2 += inc2; if (phase2 > 2.0f * PI) phase2 -= 2.0f * PI;
      phase3 += inc3; if (phase3 > 2.0f * PI) phase3 -= 2.0f * PI;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playPadChord(const float* freqs, int numFreqs, uint16_t durationMs, bool reversed) {
  // Deep, rich pad voice: each note is a 5-way detuned unison stack PLUS a
  // quiet sub-octave layer underneath for low-end weight, with a slow
  // fade-in/out for that evolving ambient-pad character. When reversed is
  // true, the envelope itself is time-flipped (release plays first, then
  // attack) so it sounds like the chord playing backward.
  if (!soundReady || systemVolume == 0) return;
  int nf = numFreqs > 4 ? 4 : numFreqs;
  const int UNISON = 5;
  const float detune[UNISON] = {0.978f, 0.990f, 1.0f, 1.010f, 1.022f};
  float phase[4][5]; float inc[4][5];
  float subPhase[4]; float subInc[4];
  for (int f = 0; f < nf; f++) {
    for (int u = 0; u < UNISON; u++) { phase[f][u] = 0.0f; inc[f][u] = 2.0f * PI * freqs[f] * detune[u] / I2S_SAMPLE_RATE; }
    subPhase[f] = 0.0f; subInc[f] = 2.0f * PI * (freqs[f] * 0.5f) / I2S_SAMPLE_RATE; // one octave below
  }
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float amp = (systemVolume / 100.0f) * (38000.0f / nf); size_t written; uint32_t sent = 0;
  const float attackFrac = 0.07f, releaseFrac = 0.18f;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      float t = (float)(sent + i) / totalSamples;
      float et = reversed ? (1.0f - t) : t; // flip the envelope's time axis
      float env;
      if (et < attackFrac) env = et / attackFrac;
      else if (et > 1.0f - releaseFrac) env = (1.0f - et) / releaseFrac;
      else env = 1.0f;
      if (env < 0.0f) env = 0.0f; if (env > 1.0f) env = 1.0f;
      env = sinf(env * (PI / 2.0f)); // ease curve -> smooth, non-clicky swell
      float mix = 0.0f;
      for (int f = 0; f < nf; f++) {
        for (int u = 0; u < UNISON; u++) { mix += sinf(phase[f][u]) * (1.0f / UNISON); phase[f][u] += inc[f][u]; if (phase[f][u] > 2.0f * PI) phase[f][u] -= 2.0f * PI; }
        mix += sinf(subPhase[f]) * 0.5f; subPhase[f] += subInc[f]; if (subPhase[f] > 2.0f * PI) subPhase[f] -= 2.0f * PI; // sub-bass weight
      }
      int16_t s = (int16_t)(mix * amp * env / 1.5f); buf[i*2] = s; buf[i*2+1] = s;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playStartupJingle() {
  // Simple ascending beeps with soft quieting at start
  if (!settingStartupJingle || !soundReady || systemVolume == 0) return;
  int originalVolume = systemVolume;
  systemVolume = max(30, systemVolume / 3); // Start very quiet
  playRichTone(523.25f, 100, 0.5f);  // C5 - soft start
  delay(30);
  systemVolume = max(50, (systemVolume * 3) / 2); // Gradually increase (1.5x)
  playRichTone(659.25f, 100, 0.7f);  // E5
  delay(30);
  systemVolume = max(70, (systemVolume * 6) / 5); // Continue increasing (1.2x)
  playRichTone(783.99f, 100, 0.85f); // G5
  delay(30);
  systemVolume = originalVolume; // Back to normal for final note
  playRichTone(1046.50f, 180, 1.0f); // C6
}
void playShutdownJingle() {
  // Simple descending beeps - the exact reverse of the startup beeps
  if (!settingStartupJingle) return;
  playRichTone(1046.50f, 100, 1.0f); // C6
  delay(30);
  playRichTone(783.99f, 100, 1.0f);  // G5
  delay(30);
  playRichTone(659.25f, 100, 1.0f);  // E5
  delay(30);
  playRichTone(523.25f, 180, 1.0f);  // C5
}
void playSleepChime() {
  // Short, gentle descending two-note tone - distinct from the full
  // shutdown/hibernate/restart chime, quick enough for a light sleep.
  playRichTone(880.0f, 90, 0.7f); delay(25); playRichTone(587.33f, 160, 0.65f);
}
void playTone(float freqHz, uint16_t durationMs) {
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase = 0.0f, phaseInc = 2.0f * PI * freqHz / I2S_SAMPLE_RATE;
  float amp = (systemVolume / 100.0f) * 16000.0f; size_t written; uint32_t sent = 0;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      float env = 1.0f - ((float)(sent + i) / totalSamples); env = env * env;
      int16_t s = (int16_t)(sinf(phase) * amp * env); buf[i*2] = s; buf[i*2+1] = s;
      phase += phaseInc; if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playToneSweep(float startFreq, float endFreq, uint16_t durationMs) {
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase = 0.0f; float amp = (systemVolume / 100.0f) * 16000.0f; size_t written; uint32_t sent = 0;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      float t = (float)(sent + i) / totalSamples;
      float freq = startFreq + (endFreq - startFreq) * t;
      float phaseInc = 2.0f * PI * freq / I2S_SAMPLE_RATE;
      float env = 1.0f - t; env = env * env;
      int16_t s = (int16_t)(sinf(phase) * amp * env); buf[i*2] = s; buf[i*2+1] = s;
      phase += phaseInc; if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playEngineTone(float freqHz, uint16_t durationMs) {
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase = 0.0f, phaseInc = freqHz / I2S_SAMPLE_RATE;
  float amp = (systemVolume / 100.0f) * 9000.0f; size_t written; uint32_t sent = 0;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      // Sawtooth wave (phase 0..1 -> -1..1) gives a grittier, more "engine-like" growl than a pure sine
      float saw = 2.0f * (phase - floorf(phase + 0.5f));
      int16_t s = (int16_t)(saw * amp); buf[i*2] = s; buf[i*2+1] = s;
      phase += phaseInc; if (phase >= 1.0f) phase -= 1.0f;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playRichTone(float freqHz, uint16_t durationMs, float ampScale) {
  // Fundamental + a quiet octave overtone, giving short UI sounds body
  // instead of a thin single-sine beep. Near-instant attack so it still
  // hits with a punchy, loud snap rather than fading in softly.
  if (!soundReady || systemVolume == 0) return;
  const int CHUNK = 128; int16_t buf[CHUNK * 2];
  uint32_t totalSamples = (uint32_t)I2S_SAMPLE_RATE * durationMs / 1000;
  float phase1 = 0.0f, phase2 = 0.0f;
  float inc1 = 2.0f * PI * freqHz / I2S_SAMPLE_RATE;
  float inc2 = 2.0f * PI * freqHz * 2.0f / I2S_SAMPLE_RATE;
  float amp = (systemVolume / 100.0f) * 24000.0f * ampScale; size_t written; uint32_t sent = 0;
  uint32_t attackSamples = I2S_SAMPLE_RATE / 500; // fixed ~2ms fade-in, just enough to avoid a click-pop
  if (attackSamples >= totalSamples) attackSamples = totalSamples / 4;
  while (sent < totalSamples) {
    int n = min((uint32_t)CHUNK, totalSamples - sent);
    for (int i = 0; i < n; i++) {
      uint32_t idx = sent + i;
      float env;
      if (idx < attackSamples) env = (float)idx / attackSamples;
      else { float t2 = (float)(idx - attackSamples) / (totalSamples - attackSamples); env = 1.0f - t2; env = env * env; }
      float mix = sinf(phase1) * 0.8f + sinf(phase2) * 0.3f;
      int16_t s = (int16_t)(mix * amp * env); buf[i*2] = s; buf[i*2+1] = s;
      phase1 += inc1; if (phase1 > 2.0f * PI) phase1 -= 2.0f * PI;
      phase2 += inc2; if (phase2 > 2.0f * PI) phase2 -= 2.0f * PI;
    }
    i2s_write(I2S_NUM_0, buf, n * 2 * sizeof(int16_t), &written, portMAX_DELAY); sent += n;
  }
}
void playClick() { playRichTone(1400, 30, 0.85f); } void playToggleOn() { playRichTone(1800, 45, 0.9f); }
void playLaser() { if (!settingGameSounds) return; playToneSweep(1900, 350, 90); }
void playEngine(int speed) { if (!settingGameSounds) return; playEngineTone(70.0f + speed * 9.0f, 20); }
void playToggleOff() { playRichTone(900, 45, 0.85f); } void playError() { playRichTone(220, 150, 1.0f); }
void playWarnSound() {
  int originalVolume = systemVolume;
  systemVolume = 90;
  playRichTone(440, 200, 1.0f);
  systemVolume = originalVolume;
}
void playSuccess() { playRichTone(660, 100, 0.9f); delay(50); playRichTone(880, 160, 1.0f); } void playKeyBeep() { playRichTone(1000, 20, 0.7f); }
void playRemoveApp() { playRichTone(700, 75, 0.9f); delay(35); playRichTone(400, 110, 0.9f); }
void playWinChime() {
  if (!settingGameSounds) return;
  // Bright ascending triumphant chime for a game win
  playBellNote(659.25f, 140, 1.0f);  // E5
  playBellNote(783.99f, 140, 1.0f);  // G5
  playBellNote(1046.50f, 140, 1.0f); // C6
  playBellNote(1318.51f, 420, 1.1f); // E6 (held)
}
void playJumpChime() {
  if (!settingGameSounds) return;
  // Short bright "hop" for a checkers jump/capture
  playBellNote(880.0f, 90, 1.0f);    // A5
  playBellNote(1174.66f, 150, 1.0f); // D6
}
void uiClick() { if (settingClickSounds) playClick(); }

// ---------- I2C & Touch ----------
void i2cScan() { Serial.println("Scanning I2C..."); for (uint8_t addr = 1; addr < 127; addr++) { Wire.beginTransmission(addr); if (Wire.endTransmission() == 0) Serial.printf("Found 0x%02X\n", addr); } }
bool ft6336AutoDetect() { for (uint8_t a : FT6336_CANDIDATE_ADDRS) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { FT6336_ADDR = a; return true; } } return false; }
uint8_t ft6336ReadReg(uint8_t reg) { Wire.beginTransmission(FT6336_ADDR); Wire.write(reg); if (Wire.endTransmission(false) != 0) return 0xFF; Wire.requestFrom((int)FT6336_ADDR, 1); return Wire.available() ? Wire.read() : 0xFF; }
bool ft6336GetTouch(uint16_t &x, uint16_t &y) {
  uint8_t points = ft6336ReadReg(FT6336_REG_TD_STATUS) & 0x0F; if (points == 0 || points > 2) return false;
  uint8_t xh = ft6336ReadReg(FT6336_REG_P1_XH), xl = ft6336ReadReg(FT6336_REG_P1_XL);
  uint8_t yh = ft6336ReadReg(FT6336_REG_P1_YH), yl = ft6336ReadReg(FT6336_REG_P1_YL);
  x = ((yh & 0x0F) << 8) | yl; y = 319 - (((xh & 0x0F) << 8) | xl); return true;
}

// ---------- IMU (Gyro/Accel) ----------
void imuInit() {
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    imuAddr = 0x68; imuAvailable = true; Serial.println("IMU found at 0x68"); return;
  }
  Wire.beginTransmission(0x6A);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(0x6A); Wire.write(0x10); Wire.write(0x60); Wire.endTransmission();
    imuAddr = 0x6A; imuAvailable = true; Serial.println("IMU found at 0x6A"); return;
  }
  imuAvailable = false; Serial.println("No IMU found");
}
void imuReadAccel() {
  if (!imuAvailable) return;
  if (imuAddr == 0x68) {
    Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((int)0x68, 6);
    if (Wire.available() >= 2) { uint8_t xh = Wire.read(); uint8_t xl = Wire.read(); accelX = (xh << 8) | xl; }
  } else if (imuAddr == 0x6A) {
    Wire.beginTransmission(0x6A); Wire.write(0x28 | 0x80); Wire.endTransmission(false);
    Wire.requestFrom((int)0x6A, 6);
    if (Wire.available() >= 2) { uint8_t xl = Wire.read(); uint8_t xh = Wire.read(); accelX = (xh << 8) | xl; }
  }
}

// ---------- Raw SPI LCD ----------
inline void lcdCmd(uint8_t cmd) { digitalWrite(PIN_LCD_DC, LOW); lcdSPI.transfer(cmd); }
inline void lcdData(uint8_t data) { digitalWrite(PIN_LCD_DC, HIGH); lcdSPI.transfer(data); }
void lcdData16(uint16_t d) { digitalWrite(PIN_LCD_DC, HIGH); lcdSPI.transfer(d >> 8); lcdSPI.transfer(d & 0xFF); }
void lcdCmdData(uint8_t cmd, const uint8_t *data, size_t len) { lcdCmd(cmd); for (size_t i = 0; i < len; i++) lcdData(data[i]); }
void st7796Init() {
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  lcdCmd(0x01); delay(150); lcdCmd(0x11); delay(120);
  uint8_t madctl = 0x28; lcdCmdData(0x36, &madctl, 1); uint8_t colmod = 0x55; lcdCmdData(0x3A, &colmod, 1);
  uint8_t d[] = {0xC3}; lcdCmdData(0xC5, d, 1); lcdCmd(0x21); delay(10); lcdCmd(0x13); delay(10); lcdCmd(0x29); delay(50);
  lcdSPI.endTransaction();
}
void lcdSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  lcdCmd(0x2A); lcdData16(x0); lcdData16(x1); lcdCmd(0x2B); lcdData16(y0); lcdData16(y1); lcdCmd(0x2C); lcdSPI.endTransaction();
}
void lcdFillScreen(uint16_t color) {
  // Mirror to PSRAM framebuffer
  if (lcdFB) { uint32_t c32 = ((uint32_t)color << 16) | color; uint32_t *fb32 = (uint32_t*)lcdFB; uint32_t cnt = ((uint32_t)LCD_WIDTH * LCD_HEIGHT) / 2; for (uint32_t i = 0; i < cnt; i++) fb32[i] = c32; }
  lcdSetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1); lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_DC, HIGH); uint8_t hi = color >> 8, lo = color & 0xFF; static uint8_t buf[1024];
  for (uint32_t i = 0; i < 512; i++) { buf[i*2]=hi; buf[i*2+1]=lo; } uint32_t total = (uint32_t)LCD_WIDTH * LCD_HEIGHT;
  for (uint32_t c = 0; c < total/512; c++) lcdSPI.transferBytes(buf, nullptr, 1024);
  if (total%512) lcdSPI.transferBytes(buf, nullptr, (total%512)*2); lcdSPI.endTransaction();
}
void lcdFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return; if (x+w > LCD_WIDTH) w = LCD_WIDTH-x; if (y+h > LCD_HEIGHT) h = LCD_HEIGHT-y;
  // Mirror to PSRAM framebuffer
  if (lcdFB) { for (uint16_t yy = 0; yy < h; yy++) { uint16_t *row = lcdFB + (y + yy) * LCD_WIDTH + x; for (uint16_t xx = 0; xx < w; xx++) row[xx] = color; } }
  lcdSetAddrWindow(x, y, x+w-1, y+h-1); lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_DC, HIGH); uint8_t hi = color >> 8, lo = color & 0xFF; static uint8_t buf[512];
  for (uint32_t i = 0; i < 256; i++) { buf[i*2]=hi; buf[i*2+1]=lo; } uint32_t total = (uint32_t)w * h;
  for (uint32_t c = 0; c < total/256; c++) lcdSPI.transferBytes(buf, nullptr, 512);
  if (total%256) lcdSPI.transferBytes(buf, nullptr, (total%256)*2); lcdSPI.endTransaction();
}
void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  int16_t dx = abs(x1-x0), sx = x0<x1?1:-1, dy = -abs(y1-y0), sy = y0<y1?1:-1, err = dx+dy, e2;
  while (true) { lcdFillRect(x0-1, y0-1, 3, 3, color); if (x0==x1 && y0==y1) break; e2 = 2*err;
    if (e2 >= dy) { err += dy; x0 += sx; } if (e2 <= dx) { err += dx; y0 += sy; } }
}
void lcdDrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  int16_t x = r, y = 0, err = 1-r;
  while (x >= y) { lcdFillRect(x0+x-1, y0+y-1, 3, 3, color); lcdFillRect(x0-x-1, y0+y-1, 3, 3, color);
    lcdFillRect(x0+x-1, y0-y-1, 3, 3, color); lcdFillRect(x0-x-1, y0-y-1, 3, 3, color);
    lcdFillRect(x0+y-1, y0+x-1, 3, 3, color); lcdFillRect(x0-y-1, y0+x-1, 3, 3, color);
    lcdFillRect(x0+y-1, y0-x-1, 3, 3, color); lcdFillRect(x0-y-1, y0-x-1, 3, 3, color);
    y++; if (err < 0) err += 2*y+1; else { x--; err += 2*(y-x)+1; } }
}
void lcdFillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  for (int16_t y = -r; y <= r; y++) { int16_t dx = (int16_t)sqrt((float)(r*r - y*y)); lcdFillRect(x0-dx, y0+y, dx*2+1, 1, color); }
}
void lcdFillTriangle(int16_t apexX, int16_t apexY, int16_t baseY, int16_t halfBaseW, uint16_t color) {
  int16_t h = baseY - apexY; if (h <= 0) return;
  for (int16_t y = 0; y <= h; y++) { int16_t w = (int32_t)halfBaseW * y / h; lcdFillRect(apexX - w, apexY + y, w*2+1, 1, color); }
}
void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  lcdFillRect(x, y, w, 1, color); lcdFillRect(x, y+h-1, w, 1, color);
  lcdFillRect(x, y, 1, h, color); lcdFillRect(x+w-1, y, 1, h, color);
}
// Pushes an already-modified rect of the PSRAM framebuffer mirror straight to
// the LCD, row by row. Used by partial-redraw code (e.g. Task Manager graph
// scroll) that edits lcdFB directly instead of going through lcdFillRect.
void lcdBlitRectFromFB(int16_t x, int16_t y, int16_t w, int16_t h) {
  if (!lcdFB || w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; } if (y < 0) { h += y; y = 0; }
  if (x + w > LCD_WIDTH) w = LCD_WIDTH - x; if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
  if (w <= 0 || h <= 0) return;
  lcdSetAddrWindow(x, y, x + w - 1, y + h - 1);
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_DC, HIGH);
  for (int16_t row = 0; row < h; row++) {
    lcdSPI.transferBytes((uint8_t*)&lcdFB[(size_t)(y + row) * LCD_WIDTH + x], nullptr, (size_t)w * 2);
  }
  lcdSPI.endTransaction();
}

// ---------- LCD GRAM Read (for screenshots / sketch save) ----------
void lcdReadRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t *buf) {
  if (x0 >= LCD_WIDTH || y0 >= LCD_HEIGHT || w == 0 || h == 0) return;
  if (x0 + w > LCD_WIDTH) w = LCD_WIDTH - x0;
  if (y0 + h > LCD_HEIGHT) h = LCD_HEIGHT - y0;

  // Use lower SPI speed for reliable GRAM read
  lcdSPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  lcdCmd(0x2A); lcdData16(x0); lcdData16(x0 + w - 1);
  lcdCmd(0x2B); lcdData16(y0); lcdData16(y0 + h - 1);
  lcdCmd(0x2E); // RAMRD
  digitalWrite(PIN_LCD_DC, HIGH);

  // Dummy byte
  lcdSPI.transfer(0x00);

  // Bulk read all pixel data at once (no yield — keeps LCD in read mode)
  size_t totalPixels = (size_t)w * h;
  size_t totalBytes = totalPixels * 2;
  uint8_t *raw = (uint8_t*)buf;

  // Read in 4KB chunks using full-duplex transfer
  static uint8_t zeroBuf[4096];
  memset(zeroBuf, 0, sizeof(zeroBuf));
  size_t bytesRead = 0;
  while (bytesRead < totalBytes) {
    size_t chunk = min((size_t)sizeof(zeroBuf), totalBytes - bytesRead);
    lcdSPI.transferBytes(zeroBuf, raw + bytesRead, chunk);
    bytesRead += chunk;
  }

  // Convert byte pairs to uint16_t (big-endian: hi, lo)
  for (size_t i = totalPixels; i > 0; i--) {
    buf[i-1] = (raw[(i-1)*2] << 8) | raw[(i-1)*2+1];
  }

  lcdSPI.endTransaction();
}

// ---------- Text Rendering ----------
#define FONT_FIRST 0x20
#define FONT_LAST  0x7A
const uint8_t FONT5x7[FONT_LAST - FONT_FIRST + 1][5] PROGMEM = {
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x2F,0x00,0x00,0x00},{0x00,0x03,0x00,0x03,0x00},{0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x03,0x00,0x00,0x00},
  {0x00,0x3E,0x41,0x00,0x00},{0x00,0x00,0x41,0x3E,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},
  {0x40,0x20,0x00,0x00,0x00},{0x04,0x04,0x04,0x04,0x04},{0x00,0x20,0x00,0x00,0x00},{0x10,0x08,0x04,0x02,0x01},
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x22,0x41,0x49,0x49,0x36},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x22,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x02,0x01,0x59,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E},{0x7C,0x12,0x11,0x12,0x7C},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x41,0x3E},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x41,0x49,0x3A},
  {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x40,0x40,0x3F},{0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x02,0x04,0x08,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x26,0x49,0x49,0x49,0x32},
  {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63},{0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00},
  {0x20,0x54,0x54,0x54,0x78},{0x7F,0x44,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x44},{0x38,0x44,0x44,0x44,0x7F},
  {0x38,0x54,0x54,0x54,0x58},{0x08,0x7E,0x09,0x09,0x02},{0x08,0x54,0x54,0x54,0x3C},{0x7F,0x04,0x04,0x04,0x78},
  {0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},
  {0x7C,0x04,0x78,0x04,0x78},{0x7C,0x04,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
  {0x08,0x14,0x14,0x14,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x24},{0x04,0x3F,0x44,0x40,0x20},
  {0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},
  {0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44}
};
void lcdDrawChar(int16_t x, int16_t y, char c, uint16_t color, uint8_t scale) {
  if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
  uint8_t idx = c - FONT_FIRST;
  for (uint8_t col = 0; col < 5; col++) {
    uint8_t bits = pgm_read_byte(&FONT5x7[idx][col]);
    for (uint8_t row = 0; row < 7; row++) {
      if (bits & (1 << row)) lcdFillRect(x + col*scale, y + row*scale, scale, scale, color);
    }
  }
}
uint16_t lcdDrawText(int16_t x, int16_t y, const char* text, uint16_t color, uint8_t scale) {
  int16_t startX = x; while (*text) { lcdDrawChar(x, y, *text, color, scale); x += (6 * scale); text++; } return x - startX;
}
uint16_t lcdTextWidth(const char* text, uint8_t scale) { return strlen(text) * 6 * scale; }
void lcdDrawTextCentered(int16_t boxX, int16_t boxW, int16_t y, const char* text, uint16_t color, uint8_t scale) {
  uint16_t w = lcdTextWidth(text, scale); int16_t x = boxX + (boxW - (int16_t)w) / 2; if (x < boxX) x = boxX; lcdDrawText(x, y, text, color, scale);
}
void drawWrappedText(int16_t x, int16_t y, int16_t maxWidth, const char* text, uint16_t color, uint8_t scale) {
  int16_t currX = x, currY = y;
  int16_t charW = 6 * scale; int16_t lineH = 8 * scale;
  while (*text) {
    if (*text == '\n') { currX = x; currY += lineH; text++; continue; }
    if (currX + charW > x + maxWidth) { currX = x; currY += lineH; }
    lcdDrawChar(currX, currY, *text, color, scale);
    currX += charW; text++;
  }
}

// ---------- Theme System ----------
uint16_t getThemePrimary() { switch(settingThemeIndex) { case THEME_MINT: return COL_MINT; case THEME_DARK: return 0x4208; case THEME_BLUE: return COL_TITLEBAR; case THEME_RED: return 0xC800; default: return COL_MINT; } }
uint16_t getThemeSecondary() { switch(settingThemeIndex) { case THEME_MINT: return COL_MINT_DK; case THEME_DARK: return 0x2104; case THEME_BLUE: return 0x0862; case THEME_RED: return 0x8000; default: return COL_MINT_DK; } }
uint16_t getThemeBg() { switch(settingThemeIndex) { case THEME_MINT: return 0x0180; case THEME_DARK: return COL_BLACK; case THEME_BLUE: return 0x0008; case THEME_RED: return 0x1000; default: return 0x0180; } }

// ---------- Control Bar ----------
void drawBarHomeIcon(int16_t cx, int16_t cy) { lcdDrawLine(cx-10, cy+2, cx, cy-8, COL_WHITE); lcdDrawLine(cx, cy-8, cx+10, cy+2, COL_WHITE); lcdFillRect(cx-6, cy+2, 12, 8, COL_WHITE); }
void drawBarBackIcon(int16_t cx, int16_t cy) { lcdDrawLine(cx+6, cy-8, cx-6, cy, COL_WHITE); lcdDrawLine(cx+6, cy+8, cx-6, cy, COL_WHITE); }
// Slim, single-row control bar. Icon-only buttons (W / HOME / BACK / POWER)
// so nothing has to squeeze a label under an icon in a 26px-tall strip;
// value buttons (brightness/volume/clock) show one compact centered line.
// No thick full-width top border - just a hairline accent so the bar reads
// as a distinct strip without a bright bar eating into the row above it.
void drawControlBar() {
  lcdFillRect(0, BAR_Y, LCD_WIDTH, BAR_H, getThemeSecondary());
  lcdFillRect(0, BAR_Y, LCD_WIDTH, 2, getThemePrimary());
  for (int i = 0; i < BAR_BTN_COUNT; i++) lcdFillRect(i*BAR_BTN_W+2, BAR_Y+2, BAR_BTN_W-4, BAR_H-4, getThemeSecondary());

  // START (W) - Now Green
  lcdFillRect(2, BAR_Y+2, BAR_BTN_W-4, BAR_H-4, COL_GREEN);
  lcdDrawTextCentered(0, BAR_BTN_W, BAR_Y+12, "W", COL_WHITE, 2);
  lcdDrawTextCentered(0, BAR_BTN_W, BAR_Y+BAR_H-10, "START", COL_WHITE, 1);

  // HOME
  drawBarHomeIcon(BAR_BTN_W + BAR_BTN_W/2, BAR_Y+13);
  lcdDrawTextCentered(BAR_BTN_W, BAR_BTN_W, BAR_Y+BAR_H-10, "HOME", COL_WHITE, 1);
  // BACK
  drawBarBackIcon(BAR_BTN_W*2 + BAR_BTN_W/2, BAR_Y+13);
  lcdDrawTextCentered(BAR_BTN_W*2, BAR_BTN_W, BAR_Y+BAR_H-10, "BACK", COL_WHITE, 1);
  // POWER
  int pcx = BAR_BTN_W*3 + BAR_BTN_W/2; int pcy = BAR_Y + 15;
  lcdDrawCircle(pcx, pcy, 8, COL_WHITE); lcdDrawLine(pcx, pcy-10, pcx, pcy-2, COL_WHITE);
  lcdDrawTextCentered(BAR_BTN_W*3, BAR_BTN_W, BAR_Y+BAR_H-10, "POWER", COL_WHITE, 1);
  // BRIGHTNESS -
  char bStr[6]; snprintf(bStr, sizeof(bStr), "%d%%", settingBrightness);
  lcdDrawTextCentered(BAR_BTN_W*4, BAR_BTN_W, BAR_Y+4, "B-", COL_WHITE, 1);
  lcdDrawTextCentered(BAR_BTN_W*4, BAR_BTN_W, BAR_Y+BAR_H-10, bStr, COL_YELLOW, 1);
  // BRIGHTNESS +
  lcdDrawTextCentered(BAR_BTN_W*5, BAR_BTN_W, BAR_Y+4, "B+", COL_WHITE, 1);
  lcdDrawTextCentered(BAR_BTN_W*5, BAR_BTN_W, BAR_Y+BAR_H-10, bStr, COL_YELLOW, 1);
  // VOLUME -
  char vStr[6]; snprintf(vStr, sizeof(vStr), "%d%%", systemVolume);
  lcdDrawTextCentered(BAR_BTN_W*6, BAR_BTN_W, BAR_Y+4, "V-", COL_WHITE, 1);
  lcdDrawTextCentered(BAR_BTN_W*6, BAR_BTN_W, BAR_Y+BAR_H-10, vStr, COL_YELLOW, 1);
  // VOLUME + (with clock)
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 100)) {
    char timeStr[8];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
    lcdDrawTextCentered(BAR_BTN_W*7, BAR_BTN_W, BAR_Y+4, timeStr, COL_WHITE, 1);
  }
  lcdDrawTextCentered(BAR_BTN_W*7, BAR_BTN_W, BAR_Y+BAR_H-10, vStr, COL_YELLOW, 1);
}

// Navigate back one step in UI hierarchy
void goBack() {
  uiClick();
  if (currentMode == MODE_START_MENU) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_SETTINGS_PRIVACY) {
    currentMode = MODE_SETTINGS; drawSettingsMain(); // was jumping straight to launcher, skipping SETTINGS
  } else if (currentMode == MODE_SETTINGS_PREFS) {
    currentMode = MODE_SETTINGS; drawSettingsMain(); // was jumping straight to launcher, skipping SETTINGS
  } else if (currentMode == MODE_SETTINGS_SOUND) {
    currentMode = MODE_SETTINGS; drawSettingsMain();
  } else if (currentMode == MODE_SETTINGS_TIME) {
    currentMode = MODE_SETTINGS; drawSettingsMain();
  } else if (currentMode == MODE_GMAIL_SIGNIN) {
    currentMode = MODE_SETTINGS_PREFS; drawSettingsPrefs();
  } else if (currentMode == MODE_EMAIL_INBOX) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_EMAIL_COMPOSE) {
    currentMode = MODE_EMAIL_INBOX; drawEmailInbox();
  } else if (currentMode == MODE_SETTINGS_SAVER) {
    // Return to wherever this screen was actually opened from (PREFERENCES
    // or the SETTINGS main menu), not always PREFERENCES.
    currentMode = settingsSaverReturnMode;
    if (settingsSaverReturnMode == MODE_SETTINGS) drawSettingsMain(); else drawSettingsPrefs();
  } else if (currentMode == MODE_WIFI_PASSWORD) {
    currentMode = MODE_WIFI; drawWiFiScreen();
  } else if (currentMode == MODE_WIFI) {
    currentMode = MODE_SETTINGS; drawSettingsMain();
  } else if (currentMode == MODE_SETTINGS) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_CAMERA_DIALOG) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_RACING_DIFFICULTY || currentMode == MODE_SF_DIFFICULTY) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_GHOSTING_WARNING) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_IP_VIEWER) {
    currentMode = MODE_IP_EXPLORER; drawIpExplorer();
  } else if (currentMode == MODE_FILE_BROWSER) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_TASK_MANAGER) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_WSHOP) {
    currentMode = MODE_LAUNCHER; drawLauncher();
  } else if (currentMode == MODE_FILENAME_INPUT) {
    currentMode = saveReturnMode;
    if (saveReturnMode == MODE_NOTEPAD) drawNotepad();
    else if (saveReturnMode == MODE_SKETCHPAD) drawSketchPad();
    else if (saveReturnMode == MODE_FILE_BROWSER) drawFileBrowser();
  } else if (currentMode == MODE_SD_CONFIRM) {
    currentMode = MODE_FILE_BROWSER; drawFileBrowser();
  } else if (currentMode == MODE_MSGBOX) {
    currentMode = msgBoxReturnMode;
    if (currentMode == MODE_LAUNCHER) drawLauncher();
    else if (currentMode == MODE_SETTINGS_PRIVACY) drawSettingsPrivacy();
    else if (currentMode == MODE_SETTINGS_PREFS) drawSettingsPrefs();
    else if (currentMode == MODE_SETTINGS_SOUND) drawSettingsSound();
    else if (currentMode == MODE_SETTINGS_TIME) drawSettingsTime();
    else if (currentMode == MODE_GMAIL_SIGNIN) drawGmailSignin();
    else if (currentMode == MODE_EMAIL_INBOX) drawEmailInbox();
    else if (currentMode == MODE_EMAIL_COMPOSE) drawEmailCompose();
    else if (currentMode == MODE_SETTINGS_SAVER) drawSettingsSaver();
    else if (currentMode == MODE_SETTINGS) drawSettingsMain();
    else if (currentMode == MODE_WIFI) drawWiFiScreen();
    else if (currentMode == MODE_START_MENU) drawStartMenu();
    else if (currentMode == MODE_IP_EXPLORER) drawIpExplorer();
    else if (currentMode == MODE_SPACE_FIGHTERS) { if (sfGameOver) sfShowGameOver(); else sfRedrawScreen(); }
  } else {
    currentMode = MODE_LAUNCHER; drawLauncher();
  }
}
bool handleControlBarTouch(uint16_t tx, uint16_t ty) {
  if (ty < BAR_Y) return false;
  int idx = tx / BAR_BTN_W;
  uiClick();
  if (idx == 0) { // START
    currentMode = MODE_START_MENU;
    startMenuTab = 2; // Default to ALL
    appHoldIdx = -1; appHoldStart = 0; appHoldFired = false;
    drawStartMenu();
  } else if (idx == 1) { // HOME
    currentMode = MODE_LAUNCHER; cameraMode = CAM_MODE_NONE; esp_camera_deinit(); camSetPowerDown(true); drawLauncher();
  } else if (idx == 2) { // BACK
    goBack();
  } else if (idx == 3) { // POWER
    if (ipScanning) { ipScanCancelled = true; ipScanFinish(true); } // save what was found before power actions can tear down WiFi
    currentMode = MODE_POWER_MENU; drawPowerMenu();
  } else if (idx == 4) { // BRIGHTNESS -
    settingBrightness = max(10, settingBrightness - 10); analogWrite(PIN_LCD_BL, settingBrightness * 255 / 100); drawControlBar(); saveSettings();
  } else if (idx == 5) { // BRIGHTNESS +
    settingBrightness = min(100, settingBrightness + 10); analogWrite(PIN_LCD_BL, settingBrightness * 255 / 100); drawControlBar(); saveSettings();
  } else if (idx == 6) { // VOLUME -
    systemVolume = max(0, systemVolume - 10); applyVolume(); drawControlBar(); saveSettings();
  } else if (idx == 7) { // VOLUME +
    systemVolume = min(90, systemVolume + 10); applyVolume(); drawControlBar(); saveSettings();
  }
  return true;
}

// ---------- Start Menu ----------
// Returns true if app idx should be shown on the current startMenuTab
bool startMenuFilterPass(int idx) {
  if (startMenuTab == 3) return startAppRemoved[idx]; // REMOVED tab: only removed apps
  if (startAppRemoved[idx]) return false; // hide removed apps from the normal tabs
  return (startMenuTab == 2 || startAppCats[idx] == startMenuTab);
}
void drawStartMenu() {
  lcdFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT - BAR_H, 0x0000);
  int tabH = 30;
  int tabCount = 4;
  int tabW = LCD_WIDTH / tabCount;
  const char* tabNames[] = {"GAMES", "SYSTEM", "ALL", "REMOVED"};
  for (int i = 0; i < tabCount; i++) {
    uint16_t col = (startMenuTab == i) ? getThemePrimary() : getThemeSecondary();
    lcdFillRect(i * tabW, 0, tabW, tabH, col);
    lcdDrawTextCentered(i * tabW, tabW, 10, tabNames[i], COL_WHITE, 1);
  }
  int gridCols = 4;
  int gridRows = 4;
  int cellW = LCD_WIDTH / gridCols;
  int cellH = (LCD_HEIGHT - BAR_H - tabH) / gridRows;
  int startY = tabH;
  int appIdx = 0;
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      while (appIdx < startAppCount) {
        if (startMenuFilterPass(appIdx)) break;
        appIdx++;
      }
      if (appIdx >= startAppCount) {
        lcdFillRect(c * cellW+2, startY + r * cellH+2, cellW-4, cellH-4, 0x2104);
        continue;
      }
      int x = c * cellW;
      int y = startY + r * cellH;
      lcdFillRect(x+2, y+2, cellW-4, cellH-4, getThemeSecondary());
      lcdDrawTextCentered(x, cellW, y + cellH/2 - 4, startAppNames[appIdx], COL_WHITE, 1);
      if (startMenuTab == 3) {
        // "+" badge to restore the app
        int bx = x + cellW - 20, by = y + 6;
        lcdFillCircle(bx, by, 9, COL_GREEN);
        lcdFillRect(bx - 5, by - 1, 10, 2, COL_WHITE);
        lcdFillRect(bx - 1, by - 5, 2, 10, COL_WHITE);
      } else if (appHoldIdx == appIdx && !appHoldFired && appHoldStart != 0) {
        // Hold-progress indicator (red line) while the icon is being held down to remove it
        unsigned long held = millis() - appHoldStart;
        int pct = (int)min((unsigned long)100, held * 100 / 700);
        int barW = (cellW - 8) * pct / 100;
        lcdFillRect(x + 4, y + cellH - 6, barW, 3, COL_RED);
      }
      appIdx++;
    }
  }
  // Version Number
  lcdDrawText(LCD_WIDTH - 60, LCD_HEIGHT - BAR_H - 20, "V8.36", COL_GRAY, 1);
  if (startMenuTab != 3) lcdDrawText(4, LCD_HEIGHT - BAR_H - 20, "HOLD TO REMOVE", COL_GRAY, 1);
  drawControlBar();
}
void handleStartMenuTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) {
    handleControlBarTouch(tx, ty);
    return;
  }
  int tabH = 30;
  int tabCount = 4;
  int tabW = LCD_WIDTH / tabCount;
  if (ty < tabH) {
    int tab = tx / tabW;
    if (tab >= 0 && tab < tabCount && tab != startMenuTab) {
      startMenuTab = tab;
      appHoldIdx = -1; appHoldStart = 0; appHoldFired = false;
      uiClick();
      drawStartMenu();
    }
    return;
  }
  int gridCols = 4;
  int gridRows = 4;
  int cellW = LCD_WIDTH / gridCols;
  int cellH = (LCD_HEIGHT - BAR_H - tabH) / gridRows;
  int startY = tabH;
  int appIdx = 0;
  for (int r = 0; r < gridRows; r++) {
    for (int c = 0; c < gridCols; c++) {
      while (appIdx < startAppCount) {
        if (startMenuFilterPass(appIdx)) break;
        appIdx++;
      }
      if (appIdx >= startAppCount) break;
      int x = c * cellW;
      int y = startY + r * cellH;
      if (tx >= x && tx < x + cellW && ty >= y && ty < y + cellH) {
        if (startMenuTab == 3) {
          // Tap on the REMOVED tab instantly restores the app
          uiClick();
          startAppRemoved[appIdx] = false;
          saveSettings();
          drawStartMenu();
          return;
        }
        // Normal tabs: start hold-tracking instead of launching immediately.
        // The actual launch (tap) or removal (hold) is resolved in loop().
        appHoldIdx = appIdx;
        appHoldStart = millis();
        appHoldFired = false;
        return;
      }
      appIdx++;
    }
  }
}
// Launches the app at startAppModes[appIdx] — called when a start-menu tap (not a hold) completes.
void launchStartMenuApp(int appIdx) {
  uiClick();
  int mode = startAppModes[appIdx];
  if (mode == MODE_CAMERA_DIALOG) {
    if (!settingCameraAccess) { playError(); showMessageBox(MSG_CRIT, "ACCESS DENIED", "CAMERA ACCESS IS", "DISABLED IN SETTINGS", MODE_START_MENU); return; }
  }
  currentMode = (AppMode)mode;
  if (mode == MODE_TICTACTOE || mode == MODE_TICTACTOE_AI) resetGame();
  else if (mode == MODE_SETTINGS) drawSettingsMain();
  else if (mode == MODE_SETTINGS_SOUND) drawSettingsSound();
  else if (mode == MODE_SETTINGS_TIME) drawSettingsTime();
  else if (mode == MODE_GMAIL_SIGNIN) drawGmailSignin();
  else if (mode == MODE_WIFI) drawWiFiScreen();
  else if (mode == MODE_CLOCK) drawClock();
  else if (mode == MODE_SKETCHPAD) drawSketchPad();
  else if (mode == MODE_NOTEPAD) drawNotepad();
  else if (mode == MODE_CAMERA_DIALOG) drawCameraDialog();
  else if (mode == MODE_CHECKERS || mode == MODE_CHECKERS_AI) { initCheckers(); drawCheckersBoard(); }
  else if (mode == MODE_MINESWEEPER) { initMinesweeper(); drawMinesweeper(); }
  else if (mode == MODE_RACING_DIFFICULTY) drawRacingDifficultyDialog();
  else if (mode == MODE_IP_EXPLORER) drawIpExplorer();
  else if (mode == MODE_SF_DIFFICULTY) drawSFDifficultyDialog();
  else if (mode == MODE_DRIVER) initDriver();
  else if (mode == MODE_FILE_BROWSER) drawFileBrowser();
  else if (mode == MODE_EMAIL_INBOX) drawEmailInbox();
  else if (mode == MODE_TASK_MANAGER) drawTaskManager();
  else if (mode == MODE_WSHOP) drawWShop();
}

// ---------- Power Menu ----------
#define PM_W 280
#define PM_H 220
#define PM_X ((LCD_WIDTH - PM_W) / 2)
#define PM_Y ((LCD_HEIGHT - PM_H) / 2)
void drawPowerMenu() {
  lcdFillScreen(0x0000);
  lcdFillRect(PM_X, PM_Y, PM_W, PM_H, COL_GRAY);
  lcdFillRect(PM_X, PM_Y, PM_W, 2, COL_WHITE); lcdFillRect(PM_X, PM_Y, 2, PM_H, COL_WHITE);
  lcdFillRect(PM_X, PM_Y+PM_H-2, PM_W, 2, COL_DKGRAY); lcdFillRect(PM_X+PM_W-2, PM_Y, 2, PM_H, COL_DKGRAY);
  lcdFillRect(PM_X, PM_Y, PM_W, 24, getThemePrimary());
  lcdDrawText(PM_X + 6, PM_Y + 7, "POWER MENU", COL_WHITE, 1);
  int btnW = 120, btnH = 35, startX = PM_X + 20, startY = PM_Y + 40, gap = 15;
  lcdFillRect(startX, startY, btnW, btnH, COL_RED); lcdFillRect(startX+2, startY+2, btnW-4, btnH-4, getThemeBg());
  lcdDrawTextCentered(startX, btnW, startY + 12, "SHUTDOWN", COL_WHITE, 1);
  lcdFillRect(startX + btnW + gap, startY, btnW, btnH, COL_BLUE); lcdFillRect(startX + btnW + gap + 2, startY+2, btnW-4, btnH-4, getThemeBg());
  lcdDrawTextCentered(startX + btnW + gap, btnW, startY + 12, "HIBERNATE", COL_WHITE, 1);
  lcdFillRect(startX, startY + btnH + gap, btnW, btnH, COL_MINT); lcdFillRect(startX+2, startY + btnH + gap + 2, btnW-4, btnH-4, getThemeBg());
  lcdDrawTextCentered(startX, btnW, startY + btnH + gap + 12, "SLEEP", COL_WHITE, 1);
  lcdFillRect(startX + btnW + gap, startY + btnH + gap, btnW, btnH, COL_YELLOW); lcdFillRect(startX + btnW + gap + 2, startY + btnH + gap + 2, btnW-4, btnH-4, getThemeBg());
  lcdDrawTextCentered(startX + btnW + gap, btnW, startY + btnH + gap + 12, "RESTART", COL_WHITE, 1);
  int cancelW = 100, cancelX = PM_X + (PM_W - cancelW) / 2, cancelY = PM_Y + PM_H - 45;
  lcdFillRect(cancelX, cancelY, cancelW, 30, COL_GRAY); lcdFillRect(cancelX+2, cancelY+2, cancelW-4, 26, getThemeBg());
  lcdDrawTextCentered(cancelX, cancelW, cancelY + 10, "CANCEL", COL_WHITE, 1);
}
bool handlePowerMenuTouch(uint16_t tx, uint16_t ty) {
  int btnW = 120, btnH = 35, startX = PM_X + 20, startY = PM_Y + 40, gap = 15;
  if (tx >= startX && tx < startX + btnW && ty >= startY && ty < startY + btnH) {
    uiClick(); prefs.begin("wave_os", false); prefs.putBool("hibernate", false); prefs.end();
    playShutdownJingle();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW); esp_deep_sleep_start(); return true;
  }
  if (tx >= startX + btnW + gap && tx < startX + btnW*2 + gap && ty >= startY && ty < startY + btnH) {
    uiClick(); saveSettings(); prefs.begin("wave_os", false); prefs.putBool("hibernate", true);
    prefs.putInt("hib_mode", currentMode); prefs.putInt("hib_cam", cameraMode); prefs.end();
    playShutdownJingle();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW); esp_deep_sleep_start(); return true;
  }
  if (tx >= startX && tx < startX + btnW && ty >= startY + btnH + gap && ty < startY + btnH*2 + gap) {
    uiClick(); playSleepChime(); esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW); esp_light_sleep_start(); return true;
  }
  if (tx >= startX + btnW + gap && tx < startX + btnW*2 + gap && ty >= startY + btnH + gap && ty < startY + btnH*2 + gap) {
    uiClick(); playShutdownJingle(); ESP.restart(); return true;
  }
  int cancelW = 100, cancelX = PM_X + (PM_W - cancelW) / 2, cancelY = PM_Y + PM_H - 45;
  if (tx >= cancelX && tx < cancelX + cancelW && ty >= cancelY && ty < cancelY + 30) {
    uiClick(); currentMode = MODE_LAUNCHER; drawLauncher(); return true;
  }
  return false;
}

// ---------- Splash Screen ----------
void drawSplash() {
  lcdFillScreen(COL_BLACK);
  for (int i = 0; i < 60; i++) { uint16_t alpha = (i * 255) / 60; uint8_t r = 0, g = (alpha * 200) / 255, b = (alpha * 100) / 255; uint16_t col = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3); lcdFillRect(0, i, LCD_WIDTH, 1, col); lcdFillRect(0, LCD_HEIGHT - 1 - i, LCD_WIDTH, 1, col); }
  int logoCX = LCD_WIDTH / 2, logoCY = LCD_HEIGHT / 2 - 30;
  for (int x = -80; x <= 80; x++) { float wave1 = sinf(x * 0.08f) * 15; float wave2 = sinf(x * 0.12f + 1.0f) * 10; lcdFillRect(logoCX + x, logoCY + (int)wave1, 2, 3, COL_MINT); lcdFillRect(logoCX + x, logoCY + 20 + (int)wave2, 2, 2, COL_MINT_DK); }
  int wX = logoCX - 40, wY = logoCY - 50; lcdDrawLine(wX, wY, wX+10, wY+30, COL_MINT); lcdDrawLine(wX+10, wY+30, wX+20, wY+10, COL_MINT); lcdDrawLine(wX+20, wY+10, wX+30, wY+30, COL_MINT); lcdDrawLine(wX+30, wY+30, wX+40, wY, COL_MINT);
  lcdDrawCircle(logoCX - 100, logoCY, 8, COL_MINT); lcdDrawCircle(logoCX + 100, logoCY, 8, COL_MINT); lcdFillCircle(logoCX - 100, logoCY, 4, COL_MINT_LT); lcdFillCircle(logoCX + 100, logoCY, 4, COL_MINT_LT);
  lcdDrawTextCentered(0, LCD_WIDTH, logoCY + 60, "WAVE OS", COL_MINT, 3); lcdDrawTextCentered(0, LCD_WIDTH, logoCY + 90, "V8.36", COL_MINT_DK, 1);
  lcdFillRect(LCD_WIDTH/2 - 80, LCD_HEIGHT - 60, 160, 8, 0x2104); lcdFillRect(LCD_WIDTH/2 - 78, LCD_HEIGHT - 58, 156, 4, COL_MINT_DK);
  lcdDrawTextCentered(0, LCD_WIDTH, LCD_HEIGHT - 30, "POWERED BY ESP_CAPABLE", COL_WHITE, 1);
}
void updateSplash() {
  unsigned long elapsed = millis() - splashStartTime; int progress = min(156, (int)(elapsed / 20));
  lcdFillRect(LCD_WIDTH/2 - 78, LCD_HEIGHT - 58, progress, 4, COL_MINT);
  if (elapsed > 3000) {
    if (!firstBootCompleted) {
      currentMode = MODE_GMAIL_SIGNIN;
      drawGmailSignin();
    } else if (pinEnabled) {
      pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); currentMode = MODE_PIN_ENTRY; drawPinEntryScreen();
    } else {
      currentMode = MODE_LAUNCHER; drawLauncher(); playDesktopJingleOnce();
    }
  }
}

// ---------- PIN Entry System ----------
void drawPinEntryScreen() {
  lcdFillScreen(getThemeBg());
  const char* title = pinSetupMode ? (pinNewLen > 0 ? "CONFIRM PIN" : "SET NEW PIN") : "ENTER PIN";
  lcdDrawTextCentered(0, LCD_WIDTH, 8, title, COL_WHITE, 2);
  int dotStartX = LCD_WIDTH/2 - 60;
  for (int i = 0; i < 4; i++) { int cx = dotStartX + i * 40 + 20, cy = 34; if (i < pinEntryLen) lcdFillCircle(cx, cy, 12, COL_MINT); else lcdDrawCircle(cx, cy, 12, COL_GRAY); }
  int btnSize = 48, btnGap = 6;
  int padStartX = LCD_WIDTH/2 - (3*btnSize + 2*btnGap)/2, padStartY = 54;
  const char* labels[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "DEL", "0", "OK"};
  for (int i = 0; i < 12; i++) { int row = i / 3, col = i % 3; int bx = padStartX + col * (btnSize + btnGap), by = padStartY + row * (btnSize + btnGap);
    bool isWide = (labels[i][0] == 'D' || labels[i][0] == 'O');
    lcdFillRect(bx, by, btnSize, btnSize, getThemeSecondary()); lcdFillRect(bx+2, by+2, btnSize-4, btnSize-4, getThemePrimary()); lcdDrawTextCentered(bx, btnSize, by + btnSize/2 - 4, labels[i], COL_WHITE, isWide ? 1 : 2); }
  if (pinSetupMode) drawControlBar();
}
bool handlePinEntryTouch(uint16_t tx, uint16_t ty) {
  if (pinSetupMode && ty >= BAR_Y) { handleControlBarTouch(tx, ty); return true; }
  int btnSize = 48, btnGap = 6;
  int padStartX = LCD_WIDTH/2 - (3*btnSize + 2*btnGap)/2, padStartY = 54;
  char keys[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#'};
  for (int i = 0; i < 12; i++) { int row = i / 3, col = i % 3; int bx = padStartX + col * (btnSize + btnGap), by = padStartY + row * (btnSize + btnGap);
    if (tx >= bx && tx < bx + btnSize && ty >= by && ty < by + btnSize) { playKeyBeep();
      if (keys[i] == '*') { if (pinEntryLen > 0) { pinEntryLen--; pinEntry[pinEntryLen] = 0; } }
      else if (keys[i] == '#') { if (pinEntryLen == 4) {
          if (pinSetupMode) { if (pinNewLen == 0) { strncpy(pinNewCode, pinEntry, 4); pinNewCode[4] = 0; pinNewLen = 4; pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); }
            else { if (strncmp(pinNewCode, pinEntry, 4) == 0) { strncpy(pinCode, pinNewCode, 4); pinCode[4] = 0; pinEnabled = true; pinSetupMode = false; pinNewLen = 0; playSuccess(); saveSettings(); showMessageBox(MSG_INFO, "PIN SET", "PIN SAVED SUCCESSFULLY", "", MODE_SETTINGS_PRIVACY); }
              else { playError(); pinNewLen = 0; pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); showMessageBox(MSG_WARN, "PIN MISMATCH", "PINS DO NOT MATCH", "TRY AGAIN", MODE_PIN_ENTRY); } } }
          else { if (strncmp(pinCode, pinEntry, 4) == 0) { pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); playSuccess(); currentMode = MODE_LAUNCHER; drawLauncher(); playDesktopJingleOnce(); }
            else { playError(); pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); showMessageBox(MSG_WARN, "WRONG PIN", "INCORRECT PIN CODE", "TRY AGAIN", MODE_PIN_ENTRY); } } } }
      else { if (pinEntryLen < 4) { pinEntry[pinEntryLen] = keys[i]; pinEntryLen++; pinEntry[pinEntryLen] = 0; } }
      drawPinEntryScreen(); return true; } } return false;
}

// ---------- WiFi System ----------
void drawWiFiScreen() {
  if (WiFi.status() == WL_CONNECTED && !wifiConnected) { wifiConnected = true; wifiConnectedSSID = wifiTargetSSID; ntpSyncNow(); }
  else if (WiFi.status() != WL_CONNECTED && wifiConnected) { wifiConnected = false; }
  lcdFillScreen(getThemeBg()); lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary()); lcdDrawText(10, 12, "WIFI SETTINGS", COL_WHITE, 2);
  int statusY = 45;
  if (wifiConnected) { lcdFillRect(20, statusY, LCD_WIDTH-40, 40, 0x0A20); lcdFillRect(20, statusY, LCD_WIDTH-40, 2, COL_GREEN); lcdDrawText(30, statusY + 8, "CONNECTED TO:", COL_WHITE, 1); lcdDrawText(30, statusY + 22, wifiConnectedSSID.c_str(), COL_GREEN, 1);
    IPAddress ip = WiFi.localIP(); char ipStr[20]; snprintf(ipStr, sizeof(ipStr), "IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]); lcdDrawText(LCD_WIDTH - 150, statusY + 22, ipStr, COL_GRAY, 1); }
  else { lcdFillRect(20, statusY, LCD_WIDTH-40, 40, 0x2004); lcdFillRect(20, statusY, LCD_WIDTH-40, 2, COL_RED); lcdDrawText(30, statusY + 8, "NOT CONNECTED", COL_WHITE, 1); lcdDrawText(30, statusY + 22, "TAP SCAN TO FIND NETWORKS", COL_GRAY, 1); }
  int btnY = 95; lcdFillRect(30, btnY, 200, 35, getThemePrimary()); lcdFillRect(32, btnY+2, 196, 31, getThemeSecondary()); lcdDrawTextCentered(30, 200, btnY + 12, "SCAN NETWORKS", COL_WHITE, 1);
  if (wifiConnected) { lcdFillRect(250, btnY, 200, 35, COL_RED); lcdFillRect(252, btnY+2, 196, 31, 0x8000); lcdDrawTextCentered(250, 200, btnY + 12, "DISCONNECT", COL_WHITE, 1); }
  int listY = 140; lcdDrawText(30, listY, "AVAILABLE NETWORKS:", COL_GRAY, 1); listY += 14;
  int visibleCount = 5; int rowH = 22;
  if (wifiScanState == 1) { lcdDrawTextCentered(0, LCD_WIDTH-60, listY + 40, "SCANNING...", COL_YELLOW, 2); }
  else if (wifiNetworkCount > 0) {
    for (int i = 0; i < visibleCount; i++) { int idx = wifiScrollOffset + i; if (idx >= wifiNetworkCount) break; int rowY = listY + i * rowH;
      lcdFillRect(30, rowY, LCD_WIDTH-90, rowH-2, 0x2104); int rssi = wifiRSSIs[idx]; int bars = (rssi > -50) ? 4 : (rssi > -60) ? 3 : (rssi > -70) ? 2 : 1;
      for (int b = 0; b < bars; b++) lcdFillRect(LCD_WIDTH - 100 + b*6, rowY + 12 - b*3, 4, b*3 + 4, COL_GREEN);
      lcdDrawText(40, rowY + 6, wifiSSIDs[idx].c_str(), COL_WHITE, 1); }
    if (wifiNetworkCount > visibleCount) { int scrollBtnX = LCD_WIDTH - 45;
      lcdFillRect(scrollBtnX, listY, 30, 20, getThemePrimary()); lcdDrawTextCentered(scrollBtnX, 30, listY + 6, "^", COL_WHITE, 1);
      lcdFillRect(scrollBtnX, listY + (visibleCount * rowH) - 20, 30, 20, getThemePrimary()); lcdDrawTextCentered(scrollBtnX, 30, listY + (visibleCount * rowH) - 14, "v", COL_WHITE, 1); }
  } else if (wifiScanState == 2) { lcdDrawTextCentered(0, LCD_WIDTH-60, listY + 40, "NO NETWORKS FOUND", COL_GRAY, 1); }
  drawControlBar();
}
void startWifiScan() {
  wifiScanState = 1; drawWiFiScreen();
  if (WiFi.getMode() != WIFI_STA) { WiFi.mode(WIFI_STA); delay(100); }
  // Stop any in-progress connection attempt (e.g. to a previously configured
  // network that isn't currently in range). Leaving that attempt running
  // keeps the radio busy and makes scanNetworks() fail/return a negative
  // "busy" code, which was being misread as "0 networks found".
  if (!wifiConnected) { WiFi.disconnect(); delay(100); }
  wifiNetworkCount = 0; wifiScrollOffset = 0;
  int16_t n = WiFi.scanNetworks(false, false, false, 300);
  if (n < 0) {
    // Scan was still busy/failed (WIFI_SCAN_FAILED/WIFI_SCAN_RUNNING) - retry once
    Serial.printf("WiFi scan busy/failed (%d), retrying...\n", n);
    delay(200);
    n = WiFi.scanNetworks(false, false, false, 300);
  }
  if (n > 0) { for (int i = 0; i < min((int)n, MAX_WIFI_NETWORKS); i++) { wifiSSIDs[i] = WiFi.SSID(i); wifiRSSIs[i] = WiFi.RSSI(i); wifiNetworkCount++; } }
  else { Serial.printf("WiFi scan result: %d\n", n); }
  WiFi.scanDelete(); wifiScanState = 2; drawWiFiScreen();
  // Restore the connection attempt to the configured network, if any, now
  // that scanning is done, so we still connect once it comes back in range.
  if (!wifiConnected && wifiTargetSSID.length() > 0) { WiFi.begin(wifiTargetSSID.c_str(), wifiPassword.c_str()); }
}
void drawWifiPasswordScreen() {
  lcdFillScreen(getThemeBg()); lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary()); lcdDrawText(10, 12, "ENTER PASSWORD", COL_WHITE, 2);
  lcdDrawTextCentered(0, LCD_WIDTH, 45, "NETWORK:", COL_GRAY, 1); lcdDrawTextCentered(0, LCD_WIDTH, 60, wifiTargetSSID.c_str(), COL_WHITE, 1);
  lcdFillRect(40, 80, LCD_WIDTH-80, 25, 0x2104); lcdFillRect(42, 82, LCD_WIDTH-84, 21, COL_BLACK);
  char typed[33]; int len = wifiPassword.length(); int n = min(len, 32);
  memcpy(typed, wifiPassword.c_str(), n); typed[n] = '\0';
  lcdDrawText(50, 88, typed, COL_WHITE, 1);
  int kbStartY = 115; int keyW = 42; int keyH = 42; int startX = 30;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 10; c++) {
      const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      int bx = startX + c * keyW; int by = kbStartY + r * keyH;
      if (strcmp(label, "SPACE") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "SPACE") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "SPACE") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, 0x2104); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "SPACE", COL_GRAY, 1);
        } continue;
      }
      if (strcmp(label, "OK") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "OK") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "OK") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, COL_RED); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "RESTART", COL_WHITE, 1);
        } continue;
      }
      uint16_t btnCol = getThemeSecondary();
      if (strcmp(label, "DEL") == 0 || strcmp(label, "SHIFT") == 0 || strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) btnCol = getThemePrimary();
      lcdFillRect(bx, by, keyW, keyH, btnCol); lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemeBg());
      char disp[8]; strncpy(disp, label, 7); disp[7] = 0;
      if (!shiftActive && disp[0] >= 'A' && disp[0] <= 'Z' && strlen(disp) == 1) disp[0] += 32;
      lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, disp, COL_WHITE, 1);
      if (strcmp(label, "SHIFT") == 0 && shiftActive) { lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemePrimary()); lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, "SHIFT", COL_WHITE, 1); }
    }
  }
  drawControlBar();
}
bool handleWifiPasswordTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return true; }
  int kbStartY = 115; int keyW = 42; int keyH = 42; int startX = 30;
  if (ty >= kbStartY) { int r = (ty - kbStartY) / keyH; int c = (tx - startX) / keyW;
    if (r >= 0 && r < 4 && c >= 0 && c < 10 && tx >= startX) { playKeyBeep(); const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      if (strcmp(label, "DEL") == 0) { if (wifiPassword.length() > 0) wifiPassword.remove(wifiPassword.length() - 1); }
      else if (strcmp(label, "SHIFT") == 0) { shiftActive = !shiftActive; }
      else if (strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) { isNumLayout = !isNumLayout; }
      else if (strcmp(label, "SPACE") == 0) { wifiPassword += ' '; }
      else if (strcmp(label, "OK") == 0) {
        if (wifiPassword.length() > 0) { prefs.begin("wave_os", false); prefs.putString("wifi_ssid", wifiTargetSSID); prefs.putString("wifi_pass", wifiPassword); prefs.end(); }
        ESP.restart();
      }
      else { char ch = label[0]; if (!shiftActive && ch >= 'A' && ch <= 'Z') ch += 32; wifiPassword += ch; if (shiftActive) shiftActive = false; }
      drawWifiPasswordScreen(); return true; } } return false;
}
bool handleWifiTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return true; }
  if (tx >= 30 && tx < 230 && ty >= 95 && ty < 130) { uiClick(); startWifiScan(); return true; }
  if (wifiConnected && tx >= 250 && tx < 450 && ty >= 95 && ty < 130) {
    uiClick(); WiFi.disconnect(true); wifiConnected = false; wifiConnectedSSID = "";
    prefs.begin("wave_os", false); prefs.putString("wifi_ssid", ""); prefs.putString("wifi_pass", ""); prefs.end();
    wifiTargetSSID = ""; wifiPassword = ""; playToggleOff(); drawWiFiScreen(); return true;
  }
  int listY = 154; int rowH = 22; int visibleCount = 5;
  for (int i = 0; i < visibleCount; i++) { int idx = wifiScrollOffset + i; if (idx >= wifiNetworkCount) break; int rowY = listY + i * rowH;
    if (tx >= 30 && tx < LCD_WIDTH-60 && ty >= rowY && ty < rowY + rowH) { uiClick(); wifiTargetSSID = wifiSSIDs[idx]; wifiPassword = ""; currentMode = MODE_WIFI_PASSWORD; drawWifiPasswordScreen(); return true; } }
  if (wifiNetworkCount > visibleCount) { int scrollBtnX = LCD_WIDTH - 45;
    if (tx >= scrollBtnX && tx < scrollBtnX + 30) {
      if (ty >= listY && ty < listY + 20 && wifiScrollOffset > 0) { wifiScrollOffset--; uiClick(); drawWiFiScreen(); return true; }
      if (ty >= listY + (visibleCount * rowH) - 20 && ty < listY + (visibleCount * rowH) && wifiScrollOffset < wifiNetworkCount - visibleCount) { wifiScrollOffset++; uiClick(); drawWiFiScreen(); return true; } } }
  return false;
}

// ---------- Settings App ----------
#define SETTINGS_TOP_Y 44
#define SETTINGS_ROW_H 29
#define SETTINGS_BACK_H 34
void drawSettingsBackBtn() { lcdFillRect(0, 0, LCD_WIDTH, SETTINGS_BACK_H, getThemePrimary()); lcdDrawText(10, 10, "< BACK", COL_WHITE, 1); }
bool settingsBackBtnTouched(uint16_t tx, uint16_t ty) { return (ty < SETTINGS_BACK_H); }
void drawToggleSwitch(int16_t x, int16_t y, bool on) { uint16_t track = on ? COL_GREEN : 0x8410; lcdFillRect(x, y, 36, 16, track); lcdFillRect(x + (on ? 20 : 2), y+2, 14, 12, COL_WHITE); }
int16_t settingsRowY(int row) { return SETTINGS_TOP_Y + row * SETTINGS_ROW_H; }

void drawSettingsMain() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "SETTINGS", COL_WHITE, 1);
  int cardW = LCD_WIDTH - 30; int cardH = 40; int startY = 42; int gap = 4; int cardX = 15;
  drawSettingsCard(cardX, startY, cardW, cardH, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "WIFI", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, wifiConnected ? wifiConnectedSSID.c_str() : "NOT CONNECTED", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  startY += cardH + gap;
  drawSettingsCard(cardX, startY, cardW, cardH, 0x4A31, getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "SOUND", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, "VOLUME, EFFECTS, ALERTS", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  startY += cardH + gap;
  drawSettingsCard(cardX, startY, cardW, cardH, 0x5A2A, getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "TIME", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, "TIMEZONE", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  startY += cardH + gap;
  drawSettingsCard(cardX, startY, cardW, cardH, 0x2965, getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "PRIVACY & SECURITY", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, "CAMERA, MIC, PIN LOCK", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  startY += cardH + gap;
  drawSettingsCard(cardX, startY, cardW, cardH, 0x39C7, getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "PREFERENCES", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, "THEME, BRIGHTNESS, ACCOUNT", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  startY += cardH + gap;
  drawSettingsCard(cardX, startY, cardW, cardH, 0x1AB1, getThemePrimary());
  lcdDrawText(cardX + 12, startY + 7, "SYSTEM", COL_WHITE, 1);
  lcdDrawText(cardX + 12, startY + 23, "SCREEN SAVER, POWER", COL_GRAY, 1);
  lcdDrawText(cardX + cardW - 20, startY + 15, ">", COL_GRAY, 1);
  // No bottom control bar here - the last card now has real breathing room
  // instead of being partially hidden behind it.
}

void handleSettingsMainTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = MODE_LAUNCHER; drawLauncher(); return; }
  int cardH = 40, gap = 4, startY = 42;
  if (ty >= startY && ty < startY + cardH) { uiClick(); currentMode = MODE_WIFI; drawWiFiScreen(); }
  else if (ty >= startY + cardH + gap && ty < startY + 2*(cardH + gap)) { uiClick(); currentMode = MODE_SETTINGS_SOUND; drawSettingsSound(); }
  else if (ty >= startY + 2*(cardH + gap) && ty < startY + 3*(cardH + gap)) { uiClick(); currentMode = MODE_SETTINGS_TIME; drawSettingsTime(); }
  else if (ty >= startY + 3*(cardH + gap) && ty < startY + 4*(cardH + gap)) { uiClick(); currentMode = MODE_SETTINGS_PRIVACY; drawSettingsPrivacy(); }
  else if (ty >= startY + 4*(cardH + gap) && ty < startY + 5*(cardH + gap)) { uiClick(); currentMode = MODE_SETTINGS_PREFS; drawSettingsPrefs(); }
  else if (ty >= startY + 5*(cardH + gap) && ty < startY + 6*(cardH + gap)) { uiClick(); settingsSaverReturnMode = MODE_SETTINGS; currentMode = MODE_SETTINGS_SAVER; drawSettingsSaver(); }
}

void drawSettingsPrivacy() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "PRIVACY & SECURITY", COL_WHITE, 1);
  const char* labels[4] = {"CAMERA ACCESS", "MICROPHONE ACCESS", "TOUCH DATA LOGGING", "AUTO-LOCK SCREEN"};
  bool* values[4] = {&settingCameraAccess, &settingMicAccess, &settingTouchLogging, &settingAutoLock};
  int cardW = LCD_WIDTH - 30; int cardX = 15; int rowH = 32; int y = 40;
  for (int i = 0; i < 4; i++) {
    drawSettingsCard(cardX, y, cardW, rowH, (i % 2) ? getThemeSecondary() : getThemeBg(), getThemePrimary());
    lcdDrawText(cardX + 12, y + 10, labels[i], COL_WHITE, 1);
    drawToggleSwitch(LCD_WIDTH - 56, y + 8, *values[i]);
    y += rowH + 3;
  }
  drawSettingsCard(cardX, y, cardW, rowH, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 10, "PIN LOCK", COL_WHITE, 1);
  if (pinEnabled) { lcdFillRect(LCD_WIDTH-86, y+6, 64, 20, COL_GREEN); lcdDrawTextCentered(LCD_WIDTH-86, 64, y+12, "ENABLED", COL_WHITE, 1); }
  else { lcdFillRect(LCD_WIDTH-86, y+6, 64, 20, COL_RED); lcdDrawTextCentered(LCD_WIDTH-86, 64, y+12, "DISABLED", COL_WHITE, 1); }
  y += rowH + 3;
  drawSettingsCard(cardX, y, cardW, rowH, getThemeBg(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 10, "CHANGE PIN", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-66, y+6, 46, 20, getThemePrimary()); lcdDrawTextCentered(LCD_WIDTH-66, 46, y+12, "SET", COL_WHITE, 1);
  y += rowH + 3;
  drawSettingsCard(cardX, y, cardW, rowH, COL_RED, COL_RED);
  lcdDrawTextCentered(cardX, cardW, y + 10, "FACTORY RESET ALL SETTINGS", COL_WHITE, 1);
  // No bottom control bar - this card used to be almost entirely hidden behind it.
}

void handleSettingsPrivacyTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = MODE_SETTINGS; drawSettingsMain(); return; }
  if (ty < 40) return;
  int rowH = 32; int yStart = 40; int row = (ty - yStart) / (rowH + 3);
  bool* values[4] = {&settingCameraAccess, &settingMicAccess, &settingTouchLogging, &settingAutoLock};
  if (row >= 0 && row < 4) {
    *values[row] = !*values[row];
    if (*values[row]) playToggleOn(); else playToggleOff();
    saveSettings(); drawSettingsPrivacy();
  } else if (row == 4) {
    if (pinEnabled) { pinEnabled = false; playToggleOff(); saveSettings(); drawSettingsPrivacy(); }
    else { uiClick(); pinSetupMode = true; pinNewLen = 0; pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); currentMode = MODE_PIN_ENTRY; drawPinEntryScreen(); }
  } else if (row == 5) {
    uiClick(); pinSetupMode = true; pinNewLen = 0; pinEntryLen = 0; memset(pinEntry, 0, sizeof(pinEntry)); currentMode = MODE_PIN_ENTRY; drawPinEntryScreen();
  } else if (row == 6) {
    uiClick(); showMessageBox(MSG_QUESTION, "FACTORY RESET?", "THIS WILL ERASE ALL", "SETTINGS. CONTINUE?", MODE_SETTINGS_PRIVACY); msgBoxResetPending = true;
  }
}

// ---------- Organized Preferences Screen ----------
void drawSettingsPrefs() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "PREFERENCES", COL_WHITE, 1);
  int cardW = LCD_WIDTH - 30; int cardX = 15; int rowH = 32; int y = 40; int gap = 3;
  // Appearance Card
  drawSettingsCard(cardX, y, cardW, rowH * 3, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "APPEARANCE", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "THEME", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-104, y+5, 80, 22, getThemePrimary());
  lcdDrawTextCentered(LCD_WIDTH-104, 80, y+12, THEME_NAMES[settingThemeIndex], COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "BRIGHTNESS", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-150, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-150, 26, y+11, "-", COL_WHITE, 1);
  char bStr[6]; snprintf(bStr, sizeof(bStr), "%d", settingBrightness);
  lcdDrawTextCentered(LCD_WIDTH-116, 40, y+11, bStr, COL_YELLOW, 1);
  lcdFillRect(LCD_WIDTH-60, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-60, 26, y+11, "+", COL_WHITE, 1);
  y += rowH + gap;
  // Gmail Card
  drawSettingsCard(cardX, y, cardW, rowH * 2, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "GMAIL", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "CONFIGURED", COL_WHITE, 1);
  if (gmailConfigured) { lcdFillRect(LCD_WIDTH-86, y+6, 64, 20, COL_GREEN); lcdDrawTextCentered(LCD_WIDTH-86, 64, y+12, "YES", COL_WHITE, 1); }
  else { lcdFillRect(LCD_WIDTH-86, y+6, 64, 20, COL_RED); lcdDrawTextCentered(LCD_WIDTH-86, 64, y+12, "NO", COL_WHITE, 1); }
  y += rowH + gap;
  // Saver Nav Card
  drawSettingsCard(cardX, y, cardW, rowH, 0x18E3, getThemePrimary());
  lcdDrawText(cardX + 12, y + 10, "SCREEN SAVER & SYSTEM", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-104, y+4, 80, 24, getThemePrimary());
  lcdDrawTextCentered(LCD_WIDTH-104, 80, y+12, "SAVER >", COL_WHITE, 1);
  // No bottom control bar.
}

void handleSettingsPrefsTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = MODE_SETTINGS; drawSettingsMain(); return; }
  if (ty < 40) return;
  int rowH = 32; int gap = 3; int y0 = 40;
  // Theme row
  int yTheme = y0 + rowH;
  if (ty >= yTheme && ty < yTheme + rowH && tx >= LCD_WIDTH-104) {
    uiClick(); settingThemeIndex = (settingThemeIndex + 1) % 4; drawSettingsPrefs(); saveSettings(); return;
  }
  // Brightness row
  int yBright = y0 + rowH * 2;
  if (ty >= yBright && ty < yBright + rowH) {
    if (tx >= LCD_WIDTH-150 && tx < LCD_WIDTH-124) { uiClick(); settingBrightness = max(0, settingBrightness - 10); analogWrite(PIN_LCD_BL, settingBrightness * 255 / 100); drawSettingsPrefs(); saveSettings(); }
    else if (tx >= LCD_WIDTH-60 && tx < LCD_WIDTH-34) { uiClick(); settingBrightness = min(100, settingBrightness + 10); analogWrite(PIN_LCD_BL, settingBrightness * 255 / 100); drawSettingsPrefs(); saveSettings(); }
    return;
  }
  // Gmail row
  int yGmail = y0 + rowH * 3 + gap + rowH;
  if (ty >= yGmail && ty < yGmail + rowH) {
    uiClick(); currentMode = MODE_GMAIL_SIGNIN; drawGmailSignin(); return;
  }
  // Saver nav row
  int ySaver = y0 + rowH * 5 + gap * 2;
  if (ty >= ySaver && ty < ySaver + rowH) {
    uiClick(); settingsSaverReturnMode = MODE_SETTINGS_PREFS; currentMode = MODE_SETTINGS_SAVER; drawSettingsSaver(); return;
  }
}

// ---------- Settings: Saver Submenu (Screen Saver + System) ----------
void drawSettingsSaver() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "SAVER & SYSTEM", COL_WHITE, 1);
  int cardW = LCD_WIDTH - 30; int cardX = 15; int rowH = 32; int y = 40;
  // Screen Saver Card
  drawSettingsCard(cardX, y, cardW, rowH * 4, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "SCREEN SAVER", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "ENABLED", COL_WHITE, 1);
  drawToggleSwitch(LCD_WIDTH - 56, y + 8, settingScreensaverEnabled);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "TIMEOUT (MIN)", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-150, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-150, 26, y+11, "-", COL_WHITE, 1);
  char tzStr[8]; snprintf(tzStr, sizeof(tzStr), "%d", settingSSTimeout);
  lcdDrawTextCentered(LCD_WIDTH-116, 40, y+11, tzStr, COL_YELLOW, 1);
  lcdFillRect(LCD_WIDTH-60, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-60, 26, y+11, "+", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "STYLE", COL_WHITE, 1);
  const char* ssNames[] = {"SMILE", "DVD", "W LOGO"};
  lcdFillRect(LCD_WIDTH-104, y+5, 80, 22, getThemePrimary());
  lcdDrawTextCentered(LCD_WIDTH-104, 80, y+12, ssNames[settingSSMode], COL_WHITE, 1);
  // No bottom control bar.
}

void handleSettingsSaverTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = settingsSaverReturnMode; (settingsSaverReturnMode == MODE_SETTINGS) ? drawSettingsMain() : drawSettingsPrefs(); return; }
  if (ty < 40) return;
  int rowH = 32; int y0 = 40;
  // SS Enabled
  int yEn = y0 + rowH;
  if (ty >= yEn && ty < yEn + rowH && tx >= LCD_WIDTH - 56 && tx < LCD_WIDTH - 20) {
    settingScreensaverEnabled = !settingScreensaverEnabled;
    if (settingScreensaverEnabled) playToggleOn(); else playToggleOff();
    drawSettingsSaver(); saveSettings(); return;
  }
  // SS Timeout
  int yTO = y0 + rowH * 2;
  if (ty >= yTO && ty < yTO + rowH) {
    if (tx >= LCD_WIDTH-150 && tx < LCD_WIDTH-124) { uiClick(); settingSSTimeout = max(1, settingSSTimeout - 1); drawSettingsSaver(); saveSettings(); }
    else if (tx >= LCD_WIDTH-60 && tx < LCD_WIDTH-34) { uiClick(); settingSSTimeout = min(60, settingSSTimeout + 1); drawSettingsSaver(); saveSettings(); }
    return;
  }
  // SS Style
  int yStyle = y0 + rowH * 3;
  if (ty >= yStyle && ty < yStyle + rowH && tx >= LCD_WIDTH-104) {
    uiClick(); settingSSMode = (settingSSMode + 1) % 3; drawSettingsSaver(); saveSettings(); return;
  }
}

// ---------- Settings: Sound Submenu ----------
void drawSettingsSound() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "SOUND", COL_WHITE, 1);
  int cardW = LCD_WIDTH - 30; int cardX = 15; int rowH = 32; int y = 40; int gap = 3;
  // System Volume Card
  drawSettingsCard(cardX, y, cardW, rowH * 2, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "SYSTEM VOLUME", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "VOLUME", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-150, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-150, 26, y+11, "-", COL_WHITE, 1);
  char vStr[6]; snprintf(vStr, sizeof(vStr), "%d", systemVolume);
  lcdDrawTextCentered(LCD_WIDTH-116, 40, y+11, vStr, COL_YELLOW, 1);
  lcdFillRect(LCD_WIDTH-60, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-60, 26, y+11, "+", COL_WHITE, 1);
  y += rowH + gap;
  // Sound Effects Card
  drawSettingsCard(cardX, y, cardW, rowH * 4, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "SOUND EFFECTS", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "CLICK SOUNDS", COL_WHITE, 1);
  drawToggleSwitch(LCD_WIDTH - 56, y + 8, settingClickSounds);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "STARTUP JINGLE", COL_WHITE, 1);
  drawToggleSwitch(LCD_WIDTH - 56, y + 8, settingStartupJingle);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "GAME SOUNDS", COL_WHITE, 1);
  drawToggleSwitch(LCD_WIDTH - 56, y + 8, settingGameSounds);
  y += rowH + gap;
  // Warning Sounds Card
  drawSettingsCard(cardX, y, cardW, rowH * 2, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "WARNING SOUNDS", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "WARN AT 90% VOL", COL_WHITE, 1);
  drawToggleSwitch(LCD_WIDTH - 56, y + 8, true);
  // No bottom control bar - this card used to be almost entirely clipped by it.
}

void handleSettingsSoundTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = MODE_SETTINGS; drawSettingsMain(); return; }
  if (ty < 40) return;
  int rowH = 32; int y0 = 40; int gap = 3;
  // Volume row
  int yVol = y0 + rowH;
  if (ty >= yVol && ty < yVol + rowH) {
    if (tx >= LCD_WIDTH-150 && tx < LCD_WIDTH-124) { uiClick(); systemVolume = max(0, systemVolume - 10); drawSettingsSound(); saveSettings(); }
    else if (tx >= LCD_WIDTH-60 && tx < LCD_WIDTH-34) { uiClick(); systemVolume = min(100, systemVolume + 10); drawSettingsSound(); saveSettings(); }
    return;
  }
  // Sound Effects card starts after Volume card + gap
  int ySndCard = y0 + rowH * 2 + gap;
  // Click Sounds row
  int yClick = ySndCard + rowH;
  if (ty >= yClick && ty < yClick + rowH && tx >= LCD_WIDTH - 56 && tx < LCD_WIDTH - 20) {
    settingClickSounds = !settingClickSounds; saveSettings(); drawSettingsSound(); if (settingClickSounds) playClick(); return;
  }
  // Startup Jingle row
  int yJingle = ySndCard + rowH * 2;
  if (ty >= yJingle && ty < yJingle + rowH && tx >= LCD_WIDTH - 56 && tx < LCD_WIDTH - 20) {
    settingStartupJingle = !settingStartupJingle; saveSettings(); drawSettingsSound(); uiClick(); return;
  }
  // Game Sounds row
  int yGameSnd = ySndCard + rowH * 3;
  if (ty >= yGameSnd && ty < yGameSnd + rowH && tx >= LCD_WIDTH - 56 && tx < LCD_WIDTH - 20) {
    settingGameSounds = !settingGameSounds; saveSettings(); drawSettingsSound(); uiClick(); return;
  }
  // Warning Sounds card starts after Sound Effects card + gap
  int yWarnCard = ySndCard + rowH * 4 + gap;
  // Warning Sounds row (fixed at 90% vol - toggle is informational only)
  int yWarn = yWarnCard + rowH;
  if (ty >= yWarn && ty < yWarn + rowH && tx >= LCD_WIDTH - 56 && tx < LCD_WIDTH - 20) {
    uiClick(); // Just provide feedback - warning sounds always play at 90%
  }
}

// ---------- Settings: Time Submenu ----------
void drawSettingsTime() {
  lcdFillScreen(getThemeBg());
  drawRoundedRect(0, 0, LCD_WIDTH, 36, 0, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "TIME", COL_WHITE, 1);
  int cardW = LCD_WIDTH - 30; int cardX = 15; int rowH = 32; int y = 40;
  // Timezone Card
  drawSettingsCard(cardX, y, cardW, rowH * 2, getThemeSecondary(), getThemePrimary());
  lcdDrawText(cardX + 12, y + 8, "TIMEZONE", COL_WHITE, 1);
  y += rowH;
  lcdDrawText(cardX + 12, y + 10, "OFFSET (HOURS)", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH-150, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-150, 26, y+11, "-", COL_WHITE, 1);
  char tzStr[8]; snprintf(tzStr, sizeof(tzStr), "%d", settingTimezoneOffset);
  lcdDrawTextCentered(LCD_WIDTH-116, 40, y+11, tzStr, COL_YELLOW, 1);
  lcdFillRect(LCD_WIDTH-60, y+5, 26, 20, 0x4208); lcdDrawTextCentered(LCD_WIDTH-60, 26, y+11, "+", COL_WHITE, 1);
  // No bottom control bar.
}

void handleSettingsTimeTouch(uint16_t tx, uint16_t ty) {
  if (ty < 36) { uiClick(); currentMode = MODE_SETTINGS; drawSettingsMain(); return; }
  if (ty < 40) return;
  int rowH = 32; int y0 = 40;
  // Timezone row
  int yTz = y0 + rowH;
  if (ty >= yTz && ty < yTz + rowH) {
    if (tx >= LCD_WIDTH-150 && tx < LCD_WIDTH-124) { uiClick(); settingTimezoneOffset = max(-12, settingTimezoneOffset - 1); drawSettingsTime(); saveSettings(); }
    else if (tx >= LCD_WIDTH-60 && tx < LCD_WIDTH-34) { uiClick(); settingTimezoneOffset = min(12, settingTimezoneOffset + 1); drawSettingsTime(); saveSettings(); }
    return;
  }
}

// ---------- Message Box (Standardized Errors) ----------
void drawCritIcon(int16_t cx, int16_t cy, int16_t r) { 
  lcdFillCircle(cx, cy, r, COL_RED); 
  int16_t k = r * 0.5; 
  for (int8_t t = -1; t <= 1; t++) { 
    lcdDrawLine(cx-k+t, cy-k, cx+k+t, cy+k, COL_WHITE); 
    lcdDrawLine(cx+k+t, cy-k, cx-k+t, cy+k, COL_WHITE); 
  } 
}
void drawWarnIcon(int16_t cx, int16_t cy, int16_t r) { int16_t apexY = cy - r, baseY = cy + r; lcdFillTriangle(cx, apexY, baseY, r + 4, COL_YELLOW); lcdDrawLine(cx, apexY, cx - (r+4), baseY, COL_BLACK); lcdDrawLine(cx, apexY, cx + (r+4), baseY, COL_BLACK); lcdDrawLine(cx - (r+4), baseY, cx + (r+4), baseY, COL_BLACK); lcdFillRect(cx-2, cy-r+10, 4, 14, COL_BLACK); lcdFillRect(cx-2, cy+r-8, 4, 4, COL_BLACK); }
void drawInfoIcon(int16_t cx, int16_t cy, int16_t r) { lcdFillCircle(cx, cy, r, COL_BLUE); lcdFillRect(cx-2, cy-r+6, 4, 4, COL_WHITE); lcdFillRect(cx-2, cy-r+14, 4, r-4, COL_WHITE); }
void drawQuestionIcon(int16_t cx, int16_t cy, int16_t r) { lcdFillCircle(cx, cy, r, 0x781F); lcdDrawTextCentered(cx - r, r * 2, cy - 8, "?", COL_WHITE, 2); }

// ---------- Shared Dialog Helpers ----------
// Rounded rectangle — draws filled rounded rect with radius r
void drawRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
  if (r > w / 2) r = w / 2; if (r > h / 2) r = h / 2; if (r < 0) r = 0;
  // Center rectangle
  lcdFillRect(x + r, y, w - 2 * r, h, color);
  // Side rectangles
  lcdFillRect(x, y + r, r, h - 2 * r, color);
  lcdFillRect(x + w - r, y + r, r, h - 2 * r, color);
  // Corners
  for (int16_t dy = 0; dy < r; dy++) {
    int16_t dx = (int16_t)(sqrt((float)(r * r - (r - dy) * (r - dy))) + 0.5f);
    lcdFillRect(x + r - dx, y + dy, dx, 1, color);
    lcdFillRect(x + r - dx, y + h - 1 - dy, dx, 1, color);
    lcdFillRect(x + w - r, y + dy, dx, 1, color);
    lcdFillRect(x + w - r, y + h - 1 - dy, dx, 1, color);
  }
}
// Standard dialog shell — dark bg, gray box with borders, colored title bar
void drawDialogShell(int16_t x, int16_t y, int16_t w, int16_t h, const char* title, uint16_t titleColor) {
  // Dark background
  lcdFillScreen(COL_BLACK);
  // Drop shadow
  lcdFillRect(x + 4, y + 4, w, h, 0x2104);
  // Main box (gray, like CRIT box)
  lcdFillRect(x, y, w, h, COL_GRAY);
  // White top/left borders, dark bottom/right borders
  lcdFillRect(x, y, w, 2, COL_WHITE);
  lcdFillRect(x, y, 2, h, COL_WHITE);
  lcdFillRect(x, y + h - 2, w, 2, COL_DKGRAY);
  lcdFillRect(x + w - 2, y, 2, h, COL_DKGRAY);
  // Title bar
  lcdFillRect(x + 2, y + 2, w - 4, 24, titleColor);
  lcdDrawText(x + 10, y + 8, title, COL_WHITE, 1);
  // Close X
  int closeX = x + w - 20, closeY = y + 6;
  lcdDrawLine(closeX, closeY, closeX + 10, closeY + 10, COL_WHITE);
  lcdDrawLine(closeX + 10, closeY, closeX, closeY + 10, COL_WHITE);
}
// Rounded card for settings UI — modernized: bigger radius + soft drop shadow
void drawSettingsCard(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t accentColor) {
  const int16_t r = 10; // was 6 - softer, more modern corner radius
  // Soft drop shadow, offset down-right
  drawRoundedRect(x + 3, y + 3, w, h, r, 0x18E3);
  drawRoundedRect(x, y, w, h, r, color);
  // Accent bar at top, following the same corner radius
  for (int16_t rr = r; rr >= 0; rr--) {
    int16_t dx = (int16_t)(sqrt((float)(r * r - rr * rr)) + 0.5f);
    lcdFillRect(x + r - dx, y + rr, dx + w - 2 * r + dx, 1, accentColor);
    lcdFillRect(x + r - dx, y + h - 1 - rr, dx + w - 2 * r + dx, 1, color); // bottom blends
  }
  lcdFillRect(x, y + r, w, 2, accentColor);
}

void playCritChime() {
  // Harsh, urgent low buzz for critical errors
  playRichTone(180, 100, 1.0f); delay(30); playRichTone(180, 100, 1.0f); delay(30); playRichTone(140, 180, 1.0f);
}
void playWarnChime() {
  // Same alert tone for Warning and Question dialogs - firm two-tone beep
  playRichTone(500, 90, 0.9f); delay(30); playRichTone(380, 150, 0.9f);
}
void showMessageBox(MsgBoxType type, const char* title, const char* line1, const char* line2, AppMode returnMode) { 
  msgBoxResetPending = false;
  msgBoxType = type; 
  strncpy(msgBoxTitle, title, sizeof(msgBoxTitle)-1); msgBoxTitle[sizeof(msgBoxTitle)-1] = 0; 
  strncpy(msgBoxLine1, line1, sizeof(msgBoxLine1)-1); msgBoxLine1[sizeof(msgBoxLine1)-1] = 0; 
  strncpy(msgBoxLine2, line2 ? line2 : "", sizeof(msgBoxLine2)-1); msgBoxLine2[sizeof(msgBoxLine2)-1] = 0; 
  msgBoxReturnMode = returnMode; 
  currentMode = MODE_MSGBOX; 
  switch (type) {
    case MSG_CRIT: playWarnSound(); break;
    case MSG_WARN: playWarnSound(); break;
    case MSG_QUESTION: playWarnChime(); break;
    case MSG_INFO: playSuccess(); break;
  }
  drawMessageBox(); 
}

void drawMessageBox() {
  lcdFillScreen(COL_BLACK);
  lcdFillRect(MBX_X + 4, MBX_Y + 4, MBX_W, MBX_H, 0x2104); // drop shadow
  lcdFillRect(MBX_X, MBX_Y, MBX_W, MBX_H, COL_GRAY); 
  lcdFillRect(MBX_X, MBX_Y, MBX_W, 2, COL_WHITE); 
  lcdFillRect(MBX_X, MBX_Y, 2, MBX_H, COL_WHITE); 
  lcdFillRect(MBX_X, MBX_Y+MBX_H-2, MBX_W, 2, COL_DKGRAY); 
  lcdFillRect(MBX_X+MBX_W-2, MBX_Y, 2, MBX_H, COL_DKGRAY);
  
  // Standardized Header Color for Errors/Critical
  uint16_t titleColor = COL_TITLEBAR;
  if (msgBoxType == MSG_CRIT || msgBoxType == MSG_WARN) titleColor = COL_RED;
  else if (msgBoxType == MSG_QUESTION) titleColor = 0x781F;
  
  lcdFillRect(MBX_X, MBX_Y, MBX_W, MBX_TITLEBAR_H, titleColor); 
  lcdDrawText(MBX_X + 6, MBX_Y + 7, msgBoxTitle, COL_WHITE, 1);
  
  switch (msgBoxType) { 
    case MSG_CRIT: drawCritIcon(MBX_ICON_CX, MBX_ICON_CY, MBX_ICON_R); break; 
    case MSG_WARN: drawWarnIcon(MBX_ICON_CX, MBX_ICON_CY, MBX_ICON_R); break; 
    case MSG_INFO: drawInfoIcon(MBX_ICON_CX, MBX_ICON_CY, MBX_ICON_R); break; 
    case MSG_QUESTION: drawQuestionIcon(MBX_ICON_CX, MBX_ICON_CY, MBX_ICON_R); break; 
  }
  
  lcdDrawText(MBX_X + 80, MBX_Y + MBX_TITLEBAR_H + 24, msgBoxLine1, COL_BLACK, 1); 
  if (msgBoxLine2[0]) lcdDrawText(MBX_X + 80, MBX_Y + MBX_TITLEBAR_H + 40, msgBoxLine2, COL_BLACK, 1);
  
  lcdFillRect(MBX_OK_X, MBX_OK_Y, MBX_OK_W, MBX_OK_H, COL_GRAY); 
  lcdFillRect(MBX_OK_X, MBX_OK_Y, MBX_OK_W, 2, COL_WHITE); 
  lcdFillRect(MBX_OK_X, MBX_OK_Y, 2, MBX_OK_H, COL_WHITE); 
  lcdFillRect(MBX_OK_X, MBX_OK_Y+MBX_OK_H-2, MBX_OK_W, 2, COL_DKGRAY); 
  lcdFillRect(MBX_OK_X+MBX_OK_W-2, MBX_OK_Y, 2, MBX_OK_H, COL_DKGRAY); 
  lcdDrawTextCentered(MBX_OK_X, MBX_OK_W, MBX_OK_Y + (MBX_OK_H-7)/2, "OK", COL_BLACK, 1);
}
bool handleMessageBoxTouch(uint16_t tx, uint16_t ty) { if (tx >= MBX_OK_X && tx < MBX_OK_X+MBX_OK_W && ty >= MBX_OK_Y && ty < MBX_OK_Y+MBX_OK_H) return true; return false; }

// ---------- Camera Dialog ----------
void drawCameraDialog() {
  drawDialogShell(DLG_X, DLG_Y, DLG_W, DLG_H, "CAMERA MODE", getThemePrimary());
  lcdFillRect(DLG_X+10, DLG_Y+30, DLG_W-20, 60, COL_WHITE); lcdDrawText(DLG_X+20, DLG_Y+40, "SELECT A CAPTURE MODE:", COL_BLACK, 1); lcdDrawText(DLG_X+20, DLG_Y+56, "LIVE, DIAGNOSTIC OR CANCEL", COL_BLACK, 1);
  int btn1_x = DLG_X + 15, btn1_y = DLG_Y + 100; lcdFillRect(btn1_x, btn1_y, BTN_W, BTN_H, COL_GREEN); lcdFillRect(btn1_x+2, btn1_y+2, BTN_W-4, BTN_H-4, COL_BLACK); lcdDrawTextCentered(btn1_x, BTN_W, btn1_y + 14, "LIVE VIEW", COL_GREEN, 1);
  int btn2_x = DLG_X + 15 + BTN_W + BTN_SPACING, btn2_y = DLG_Y + 100; lcdFillRect(btn2_x, btn2_y, BTN_W, BTN_H, COL_YELLOW); lcdFillRect(btn2_x+2, btn2_y+2, BTN_W-4, BTN_H-4, COL_BLACK); lcdDrawTextCentered(btn2_x, BTN_W, btn2_y + 14, "DIAGNOSTIC", COL_YELLOW, 1);
  int btn3_x = DLG_X + 15 + (BTN_W + BTN_SPACING) * 2, btn3_y = DLG_Y + 100; lcdFillRect(btn3_x, btn3_y, BTN_W, BTN_H, COL_RED); lcdFillRect(btn3_x+2, btn3_y+2, BTN_W-4, BTN_H-4, COL_BLACK); lcdDrawTextCentered(btn3_x, BTN_W, btn3_y + 14, "CANCEL", COL_RED, 1);
  drawControlBar();
}
bool handleDialogTouch(uint16_t tx, uint16_t ty) {
  int btn1_x = DLG_X + 15, btn1_y = DLG_Y + 100; int btn2_x = DLG_X + 15 + BTN_W + BTN_SPACING, btn2_y = DLG_Y + 100; int btn3_x = DLG_X + 15 + (BTN_W + BTN_SPACING) * 2, btn3_y = DLG_Y + 100;
  if (tx >= btn1_x && tx < btn1_x+BTN_W && ty >= btn1_y && ty < btn1_y+BTN_H) { cameraMode = CAM_MODE_LIVE; return true; }
  if (tx >= btn2_x && tx < btn2_x+BTN_W && ty >= btn2_y && ty < btn2_y+BTN_H) { cameraMode = CAM_MODE_DIAGNOSTIC; return true; }
  if (tx >= btn3_x && tx < btn3_x+BTN_W && ty >= btn3_y && ty < btn3_y+BTN_H) return true; return false;
}

// ---------- Ghosting Warning Dialog ----------
void drawGhostingWarning() {
  int bx = (LCD_WIDTH - 300) / 2;
  int by = (LCD_HEIGHT - 160) / 2;
  drawDialogShell(bx, by, 300, 160, "WARNING", COL_YELLOW);
  drawWarnIcon(bx + 40, by + 80, 20);
  lcdDrawText(bx + 80, by + 50, "YOU CANT EXIT", COL_BLACK, 1);
  lcdDrawText(bx + 80, by + 65, "FOR 30 MINS", COL_BLACK, 1);
  int btnW = 100, btnH = 30;
  int btnY = by + 110;
  lcdFillRect(bx + 20, btnY, btnW, btnH, COL_RED);
  lcdFillRect(bx + 22, btnY+2, btnW-4, btnH-4, COL_BLACK);
  lcdDrawTextCentered(bx + 20, btnW, btnY + 10, "CANCEL", COL_WHITE, 1);
  lcdFillRect(bx + 180, btnY, btnW, btnH, COL_GREEN);
  lcdFillRect(bx + 182, btnY+2, btnW-4, btnH-4, COL_BLACK);
  lcdDrawTextCentered(bx + 180, btnW, btnY + 10, "CONTINUE", COL_WHITE, 1);
}
bool handleGhostingWarningTouch(uint16_t tx, uint16_t ty) {
  int bx = (LCD_WIDTH - 300) / 2;
  int by = (LCD_HEIGHT - 160) / 2;
  int btnW = 100, btnH = 30;
  int btnY = by + 110;
  if (tx >= bx + 20 && tx < bx + 120 && ty >= btnY && ty < btnY + btnH) {
    uiClick();
    currentMode = MODE_LAUNCHER;
    drawLauncher();
    return true;
  }
  if (tx >= bx + 180 && tx < bx + 280 && ty >= btnY && ty < btnY + btnH) {
    uiClick();
    currentMode = MODE_GHOSTING;
    ghostingStartTime = millis();
    ghostingLastToggle = millis();
    ghostingState = false;
    lcdFillScreen(COL_BLACK);
    return true;
  }
  return false;
}

// ---------- Launcher (Themed Tiles + Colored Icons) ----------
// ---------- AIO Icon Fallback System ----------
// Total launcher slots. Bump this (and add entries to appLabels[] /
// appHasCustomIcon[]) whenever a new app is registered. Any slot whose
// appHasCustomIcon[] entry is false — including every slot beyond the
// last one anyone bothered to code a matrix for — automatically falls
// back to the default "W" badge + title instead of drawing nothing or
// crashing on an out-of-range array read.
#define MAX_APPS 20
const char* appLabels[MAX_APPS] = {"TTT", "TTT AI", "SKETCH", "CAMERA", "SETTINGS", "WIFI", "NOTEPAD", "CHECKERS", "CHK+AI", "MINES", "RACING", "GHOST", "CLOCK", "IP EXP", "SPACE", "DRIVER", "FILES", "EMAIL", "TASK MGR", "W SHOP"};
// Set to true for every idx that has a hand-drawn icon in the switch below (0..19 all do today).
// When you add app #20+ without drawing a matrix for it yet, just leave its slot false (or
// grow the arrays with false) and it will render as the default W-badge automatically.
bool appHasCustomIcon[MAX_APPS] = {
  true, true, true, true, true, true, true, true, true, true,
  true, true, true, true, true, true, true, true, true, true
};
// Safe label lookup — returns "APP" for any appId outside the known range
// instead of reading garbage / crashing, so unregistered slots still show text.
const char* getAppLabel(int appId) {
  if (appId < 0 || appId >= MAX_APPS || appLabels[appId] == nullptr) return "APP";
  return appLabels[appId];
}
// The default icon for any app slot that hasn't had a custom matrix/bitmap coded in yet:
// a simple "W" logo badge. The title itself is already drawn separately below the tile,
// so this only needs to render the glyph.
void drawDefaultAppIcon(int cx, int cy) {
  lcdFillCircle(cx, cy, 20, getThemePrimary());
  lcdDrawCircle(cx, cy, 20, COL_WHITE);
  lcdDrawText(cx - 7, cy - 8, "W", COL_WHITE, 2);
}
// Returns true if the launcher tile for appId should be shown (i.e. its app isn't hidden)
bool launcherAppVisible(int appId) {
  int m = launcherToStartIdx[appId];
  return !(m >= 0 && startAppRemoved[m]);
}
void drawLauncher() {
  lcdFillScreen(getThemeBg());
  uint16_t tileFill = getThemeSecondary(); uint16_t tileBorder = getThemePrimary();
  const char* labels[MAX_APPS]; for (int i = 0; i < MAX_APPS; i++) labels[i] = getAppLabel(i);
  int appId = 0;
  for (int r = 0; r < GRID_ROWS; r++) {
    for (int c = 0; c < GRID_COLS; c++) {
      while (appId < MAX_APPS && !launcherAppVisible(appId)) appId++;
      if (appId >= MAX_APPS) continue; // no more apps to place — leave remaining cells as background
      if (!appHasCustomIcon[appId]) {
        // AIO fallback path: no matrix coded for this app yet — draw default W badge + label and skip the hand-coded switch entirely
        int x = c * CELL_W, y = r * CELL_H;
        lcdFillRect(x+2, y+2, CELL_W-4, CELL_H-4, tileFill);
        drawDefaultAppIcon(x + CELL_W/2, y + CELL_H/2 - 4);
        lcdDrawTextCentered(x+2, CELL_W-4, y + CELL_H - 16, labels[appId], COL_WHITE, 1);
        appId++;
        continue;
      }
      int x = c * CELL_W, y = r * CELL_H;
      int idx = appId; // original app id — reused below by the icon-drawing switch
      lcdFillRect(x+2, y+2, CELL_W-4, CELL_H-4, tileFill);
      int cx = x + CELL_W/2, cy = y + CELL_H/2 - 4;
      if (idx == 0) { // TTT: bold double-stroke X and a shaded O, offset so both read clearly
        lcdDrawLine(cx-18, cy-18, cx-2, cy-2, COL_RED); lcdDrawLine(cx-19, cy-17, cx-3, cy-1, COL_RED);
        lcdDrawLine(cx-2, cy-18, cx-18, cy-2, COL_RED); lcdDrawLine(cx-3, cy-17, cx-19, cy-1, COL_RED);
        lcdDrawCircle(cx+12, cy+12, 11, COL_BLUE); lcdDrawCircle(cx+12, cy+12, 10, COL_BLUE);
      }
      else if (idx == 1) { // TTT AI: robot head with antenna + glowing eyes
        lcdFillRect(cx-2, cy-26, 4, 8, COL_GRAY); lcdFillCircle(cx, cy-27, 3, COL_RED);
        lcdFillRect(cx-20, cy-18, 40, 30, 0x07FF); lcdDrawRect(cx-20, cy-18, 40, 30, COL_BLACK);
        lcdFillCircle(cx-9, cy-6, 5, COL_WHITE); lcdFillCircle(cx-9, cy-6, 2, COL_BLACK);
        lcdFillCircle(cx+9, cy-6, 5, COL_WHITE); lcdFillCircle(cx+9, cy-6, 2, COL_BLACK);
        lcdFillRect(cx-8, cy+4, 16, 3, COL_BLACK);
      }
      else if (idx == 2) { // SKETCH: pencil with metal ferrule, eraser and shaded tip
        lcdFillRect(cx-8, cy-22, 16, 34, COL_YELLOW); lcdDrawRect(cx-8, cy-22, 16, 34, COL_BLACK);
        lcdFillRect(cx-8, cy+12, 16, 8, 0x07FF); lcdDrawRect(cx-8, cy+12, 16, 8, COL_BLACK);
        lcdFillRect(cx-8, cy+20, 16, 6, COL_RED); lcdDrawRect(cx-8, cy+20, 16, 6, COL_BLACK);
        lcdFillTriangle(cx, cy-30, cy-22, 8, 0xFBAE);
        lcdFillTriangle(cx, cy-34, cy-30, 3, COL_BLACK);
        lcdDrawLine(cx-4, cy-14, cx+4, cy-14, 0x8410);
      }
      else if (idx == 3) { // CAMERA: body with grip, flash and reflective lens
        lcdFillRect(cx-25, cy-15, 50, 30, COL_DKGRAY); lcdDrawRect(cx-25, cy-15, 50, 30, COL_BLACK);
        lcdFillRect(cx-10, cy-25, 20, 11, COL_DKGRAY); lcdDrawRect(cx-10, cy-25, 20, 11, COL_BLACK);
        lcdFillCircle(cx, cy, 11, COL_BLACK); lcdFillCircle(cx, cy, 8, COL_BLUE); lcdFillCircle(cx-3, cy-3, 3, COL_WHITE);
        lcdFillRect(cx+14, cy-11, 6, 5, COL_YELLOW);
      }
      else if (idx == 4) { // SETTINGS: gear with visible teeth and inset hub
        for (int a = 0; a < 8; a++) { float ang = a * (PI/4.0f); int lx = cx + (int)(19*cosf(ang)), ly = cy + (int)(19*sinf(ang)); lcdFillRect(lx-4, ly-4, 8, 8, COL_WHITE); }
        lcdFillCircle(cx, cy, 15, COL_WHITE); lcdFillCircle(cx, cy, 9, tileFill); lcdDrawCircle(cx, cy, 9, COL_WHITE); lcdDrawCircle(cx, cy, 15, 0x8410);
      }
      else if (idx == 5) { // WIFI: signal arcs fading outward + solid dot
        lcdDrawCircle(cx, cy+11, 24, 0x8410); lcdDrawCircle(cx, cy+11, 24, 0x8410);
        for (int arc = 1; arc <= 3; arc++) { lcdDrawCircle(cx, cy + 11, arc * 8, 0x07FF); lcdDrawCircle(cx, cy + 11, arc * 8 - 1, 0x07FF); }
        lcdFillCircle(cx, cy + 11, 4, COL_WHITE); lcdDrawCircle(cx, cy + 11, 4, COL_BLUE);
      }
      else if (idx == 6) { // NOTEPAD: paper with folded corner and ruled lines
        lcdFillRect(cx-16, cy-20, 32, 40, COL_YELLOW); lcdDrawRect(cx-16, cy-20, 32, 40, COL_BLACK);
        lcdFillTriangle(cx+8, cy-20, cy-12, 8, 0xE71C); lcdDrawLine(cx+16, cy-20, cx+8, cy-12, COL_BLACK);
        lcdDrawLine(cx-10, cy-8, cx+9, cy-8, COL_BLUE); lcdDrawLine(cx-10, cy+2, cx+9, cy+2, COL_BLUE); lcdDrawLine(cx-10, cy+12, cx+9, cy+12, COL_BLUE);
      }
      else if (idx == 7) { // CHECKERS: beveled board with a raised red piece
        lcdFillRect(cx-21,cy-21,20,20,COL_RED); lcdFillRect(cx+1,cy-21,20,20,COL_BLACK); lcdFillRect(cx-21,cy+1,20,20,COL_BLACK); lcdFillRect(cx+1,cy+1,20,20,COL_RED);
        lcdDrawRect(cx-21, cy-21, 42, 42, 0x8410);
        lcdFillCircle(cx-11, cy-11, 8, COL_RED); lcdDrawCircle(cx-11, cy-11, 8, COL_WHITE); lcdFillCircle(cx-13, cy-13, 2, 0xFBAE);
      }
      else if (idx == 8) { // CHECKERS + AI: same board with a glowing "AI" chip badge
        lcdFillRect(cx-21,cy-21,20,20,COL_RED); lcdFillRect(cx+1,cy-21,20,20,COL_BLACK); lcdFillRect(cx-21,cy+1,20,20,COL_BLACK); lcdFillRect(cx+1,cy+1,20,20,COL_RED);
        lcdDrawRect(cx-21, cy-21, 42, 42, 0x8410);
        lcdFillCircle(cx+22,cy-22,10,0x07FF); lcdDrawCircle(cx+22,cy-22,10,COL_WHITE); lcdDrawTextCentered(cx+12, 20, cy-27, "AI", COL_BLACK, 1);
      }
      else if (idx == 9) { // MINES: spiky mine with rim light and fuse spark
        for (int a = 0; a < 8; a++) { float ang = a * (PI/4.0f); int lx = cx + (int)(22*cosf(ang)), ly = cy + (int)(22*sinf(ang)); lcdFillRect(lx-2, ly-2, 4, 4, COL_DKGRAY); }
        lcdFillCircle(cx, cy, 14, COL_DKGRAY); lcdDrawCircle(cx, cy, 14, COL_BLACK); lcdFillCircle(cx-4, cy-4, 3, COL_WHITE);
        lcdDrawLine(cx+6, cy-14, cx+12, cy-22, COL_GRAY); lcdFillCircle(cx+13, cy-24, 3, COL_YELLOW);
      }
      else if (idx == 10) { // RACING: sporty car with windshield and racing stripe
        lcdFillRect(cx-20, cy-2, 40, 16, COL_RED); lcdDrawRect(cx-20, cy-2, 40, 16, COL_BLACK);
        lcdFillRect(cx-13, cy-12, 26, 12, 0x07FF); lcdDrawRect(cx-13, cy-12, 26, 12, COL_BLACK);
        lcdFillRect(cx-4, cy-2, 8, 16, COL_YELLOW);
        lcdFillCircle(cx-13, cy+13, 7, COL_BLACK); lcdFillCircle(cx-13, cy+13, 3, 0x8410);
        lcdFillCircle(cx+13, cy+13, 7, COL_BLACK); lcdFillCircle(cx+13, cy+13, 3, 0x8410);
      }
      else if (idx == 11) { // GHOST (clear-ghosting): black/white lockout badge with a lightning bolt
        lcdFillCircle(cx, cy, 20, COL_BLACK);
        lcdFillRect(cx, cy-19, 19, 38, COL_WHITE);
        lcdDrawCircle(cx, cy, 20, 0x8410);
        lcdDrawLine(cx+3, cy-11, cx-4, cy, COL_YELLOW); lcdDrawLine(cx-4, cy, cx+4, cy, COL_YELLOW); lcdDrawLine(cx+4, cy, cx-3, cy+11, COL_YELLOW);
      }
      else if (idx == 12) { // CLOCK: face with tick marks and two hands
        lcdFillCircle(cx, cy, 19, COL_WHITE); lcdDrawCircle(cx, cy, 19, COL_BLACK); lcdDrawCircle(cx, cy, 18, 0x8410);
        for (int a = 0; a < 12; a++) { float ang = a * (PI/6.0f); int lx1 = cx + (int)(16*sinf(ang)), ly1 = cy - (int)(16*cosf(ang)); int lx2 = cx + (int)(13*sinf(ang)), ly2 = cy - (int)(13*cosf(ang)); lcdDrawLine(lx1, ly1, lx2, ly2, COL_BLACK); }
        lcdDrawLine(cx, cy, cx, cy-11, COL_BLACK); lcdDrawLine(cx, cy, cx+8, cy+3, COL_BLACK); lcdFillCircle(cx, cy, 2, COL_RED);
      }
      else if (idx == 13) { // IP Explorer Icon: globe with lat/long grid
        lcdFillCircle(cx, cy, 16, COL_GREEN); lcdDrawCircle(cx, cy, 16, COL_BLACK);
        lcdDrawLine(cx-16, cy, cx+16, cy, COL_WHITE);
        lcdDrawLine(cx, cy-16, cx, cy+16, COL_WHITE);
        lcdDrawCircle(cx, cy, 9, COL_WHITE);
      }
      else if (idx == 14) { // Space Fighters Icon: rocket with cockpit window, fins and flame
        lcdFillTriangle(cx, cy-24, cy+10, 11, COL_GRAY); lcdDrawLine(cx, cy-24, cx-11, cy+10, COL_BLACK); lcdDrawLine(cx, cy-24, cx+11, cy+10, COL_BLACK);
        lcdFillRect(cx-9, cy+10, 18, 8, COL_GRAY); lcdDrawRect(cx-9, cy+10, 18, 8, COL_BLACK);
        lcdFillTriangle(cx-9, cy+10, cy+22, 8, COL_RED); lcdFillTriangle(cx+9, cy+10, cy+22, 8, COL_RED);
        lcdFillCircle(cx, cy-6, 6, 0x07FF); lcdDrawCircle(cx, cy-6, 6, COL_BLACK); lcdFillCircle(cx-2, cy-8, 2, COL_WHITE);
        lcdFillTriangle(cx, cy+18, cy+30, 5, COL_YELLOW);
      }
      else if (idx == 15) { // DRIVER Icon: car angled differently from RACING for visual distinction
        lcdFillRect(cx-20, cy-6, 40, 18, COL_BLUE); lcdDrawRect(cx-20, cy-6, 40, 18, COL_BLACK);
        lcdFillRect(cx-12, cy-16, 24, 12, 0x8410); lcdDrawRect(cx-12, cy-16, 24, 12, COL_BLACK);
        lcdDrawLine(cx, cy-16, cx, cy-4, COL_BLACK);
        lcdFillCircle(cx-13, cy+12, 6, COL_BLACK); lcdFillCircle(cx-13, cy+12, 2, COL_GRAY);
        lcdFillCircle(cx+13, cy+12, 6, COL_BLACK); lcdFillCircle(cx+13, cy+12, 2, COL_GRAY);
        lcdFillRect(cx+15, cy-4, 5, 3, COL_YELLOW);
      }
      else if (idx == 16) { // FILES Icon: folder with tab, shaded body and dog-ear
        lcdFillRect(cx-22, cy-14, 20, 8, 0xFFF0); lcdDrawRect(cx-22, cy-14, 20, 8, COL_BLACK);
        lcdFillRect(cx-22, cy-8, 44, 26, COL_YELLOW); lcdDrawRect(cx-22, cy-8, 44, 26, COL_BLACK);
        lcdFillRect(cx-22, cy+12, 44, 6, 0xE71C);
        lcdDrawLine(cx-8, cy-6, cx-8, cy+16, 0xC618); lcdDrawLine(cx+6, cy-6, cx+6, cy+16, 0xC618);
      }
      else if (idx == 17) { // EMAIL Icon: envelope with @ symbol
        lcdFillRect(cx-20, cy-14, 40, 28, COL_BLUE); lcdDrawRect(cx-20, cy-14, 40, 28, COL_BLACK);
        lcdFillRect(cx-20, cy-20, 40, 8, COL_BLUE); lcdDrawRect(cx-20, cy-20, 40, 8, COL_BLACK);
        lcdFillRect(cx-16, cy-16, 32, 24, COL_WHITE); lcdDrawRect(cx-16, cy-16, 32, 24, COL_BLACK);
        // @ symbol
        lcdDrawCircle(cx, cy-2, 6, COL_BLUE); lcdDrawCircle(cx, cy-2, 4, COL_WHITE);
        lcdDrawLine(cx-4, cy+2, cx+4, cy+2, COL_BLUE);
        lcdDrawLine(cx, cy-2, cx, cy+6, COL_BLUE);
      }
      else if (idx == 18) { // TASK MANAGER Icon: CPU/GPU with usage bars
        // CPU (Core 0) - green
        lcdFillRect(cx-20, cy-10, 40, 8, COL_DKGRAY); lcdDrawRect(cx-20, cy-10, 40, 8, COL_BLACK);
        lcdFillRect(cx-20, cy-10, 25, 8, COL_GREEN);
        // GPU (Core 1) - blue
        lcdFillRect(cx-20, cy+2, 40, 8, COL_DKGRAY); lcdDrawRect(cx-20, cy+2, 40, 8, COL_BLACK);
        lcdFillRect(cx-20, cy+2, 30, 8, COL_BLUE);
        // Labels
        lcdDrawText(cx-18, cy-18, "CPU", COL_WHITE, 1);
        lcdDrawText(cx-18, cy+14, "GPU", COL_WHITE, 1);
      }
      else if (idx == 19) { // W SHOP Icon: shopping bag with a "+" tag
        lcdFillRect(cx-16, cy-6, 32, 26, COL_YELLOW); lcdDrawRect(cx-16, cy-6, 32, 26, COL_BLACK);
        lcdDrawLine(cx-9, cy-6, cx-9, cy-16, COL_BLACK); lcdDrawLine(cx-9, cy-16, cx-3, cy-22, COL_BLACK);
        lcdDrawLine(cx+9, cy-6, cx+9, cy-16, COL_BLACK); lcdDrawLine(cx+9, cy-16, cx+3, cy-22, COL_BLACK);
        lcdFillCircle(cx+14, cy-16, 9, COL_GREEN); lcdDrawCircle(cx+14, cy-16, 9, COL_BLACK);
        lcdFillRect(cx+11, cy-17, 6, 2, COL_WHITE); lcdFillRect(cx+13, cy-19, 2, 6, COL_WHITE);
      }
      lcdDrawTextCentered(x+2, CELL_W-4, y + CELL_H - 16, labels[idx], COL_WHITE, 1);
      if (launcherHoldIdx == idx && !launcherHoldFired && launcherHoldStart != 0) {
        // Hold-progress indicator (red line) while the icon is being held down to remove it
        unsigned long held = millis() - launcherHoldStart;
        int pct = (int)min((unsigned long)100, held * 100 / 700);
        int barW = (CELL_W - 6) * pct / 100;
        lcdFillRect(x + 3, y + CELL_H - 5, barW, 3, COL_RED);
      }
      appId++;
    }
  }
  drawControlBar();
}
// Launches the launcher tile at idx — called when a launcher tap (not a hold) completes.
void launchLauncherApp(int idx) {
  uiClick();
  if (idx == 0) { currentMode = MODE_TICTACTOE; resetGame(); }
  else if (idx == 1) { currentMode = MODE_TICTACTOE_AI; resetGame(); }
  else if (idx == 2) { currentMode = MODE_SKETCHPAD; drawSketchPad(); }
  else if (idx == 3) { if (!settingCameraAccess) { playError(); showMessageBox(MSG_CRIT, "ACCESS DENIED", "CAMERA ACCESS IS", "DISABLED IN SETTINGS", MODE_LAUNCHER); } else { currentMode = MODE_CAMERA_DIALOG; drawCameraDialog(); } }
  else if (idx == 4) { currentMode = MODE_SETTINGS; drawSettingsMain(); }
  else if (idx == 5) { currentMode = MODE_WIFI; drawWiFiScreen(); }
  else if (idx == 6) { currentMode = MODE_NOTEPAD; drawNotepad(); }
  else if (idx == 7) { currentMode = MODE_CHECKERS; initCheckers(); drawCheckersBoard(); }
  else if (idx == 8) { currentMode = MODE_CHECKERS_AI; initCheckers(); drawCheckersBoard(); }
  else if (idx == 9) { currentMode = MODE_MINESWEEPER; initMinesweeper(); drawMinesweeper(); }
  else if (idx == 10) { currentMode = MODE_RACING_DIFFICULTY; drawRacingDifficultyDialog(); }
  else if (idx == 11) { currentMode = MODE_GHOSTING_WARNING; drawGhostingWarning(); }
  else if (idx == 12) { currentMode = MODE_CLOCK; drawClock(); }
  else if (idx == 13) { currentMode = MODE_IP_EXPLORER; drawIpExplorer(); }
  else if (idx == 14) { currentMode = MODE_SF_DIFFICULTY; drawSFDifficultyDialog(); }
  else if (idx == 15) { currentMode = MODE_DRIVER; initDriver(); }
  else if (idx == 16) { currentMode = MODE_FILE_BROWSER; drawFileBrowser(); }
  else if (idx == 17) { currentMode = MODE_EMAIL_INBOX; drawEmailInbox(); }
  else if (idx == 18) { currentMode = MODE_TASK_MANAGER; GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTaskManager}; xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10)); }
  else if (idx == 19) { openWShop(); }
}
void handleLauncherTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) return;
  int col = tx / CELL_W, row = ty / CELL_H;
  // Walk the same compacted layout drawLauncher() uses to find which appId (if any) sits at (row,col)
  int appId = 0; int foundAppId = -1;
  for (int r = 0; r < GRID_ROWS && foundAppId < 0; r++) {
    for (int c = 0; c < GRID_COLS; c++) {
      while (appId < MAX_APPS && !launcherAppVisible(appId)) appId++;
      if (appId >= MAX_APPS) break;
      if (r == row && c == col) { foundAppId = appId; break; }
      appId++;
    }
  }
  if (foundAppId < 0) return; // tapped blank grid space
  // Start hold-tracking instead of launching immediately.
  // The actual launch (tap) or removal (hold) is resolved in loop().
  launcherHoldIdx = foundAppId;
  launcherHoldStart = millis();
  launcherHoldFired = false;
}

// ---------- Retro Racing Game (Local Refresh + Arrows) ----------
#define RACING_ROAD_WIDTH 240
#define RACING_ROAD_X ((LCD_WIDTH - RACING_ROAD_WIDTH) / 2)
#define RACING_CAR_WIDTH 30
#define RACING_CAR_HEIGHT 40
#define RACING_OBSTACLE_WIDTH 30
#define RACING_OBSTACLE_HEIGHT 40
int racingCarX = LCD_WIDTH / 2;
int racingScore = 0;
int racingSpeed = 3;
int racingGameOver = false;
int racingRoadOffset = 0;
int racingObstacleY = -50;
int racingObstacleX = 0;
unsigned long racingLastUpdate = 0;
int steerInput = 0;
void drawRacingBackground() {
  lcdFillScreen(0x0010);
  lcdFillRect(RACING_ROAD_X, 0, RACING_ROAD_WIDTH, LCD_HEIGHT - BAR_H, 0x4208);
  lcdFillRect(RACING_ROAD_X, 0, 4, LCD_HEIGHT - BAR_H, COL_WHITE);
  lcdFillRect(RACING_ROAD_X + RACING_ROAD_WIDTH - 4, 0, 4, LCD_HEIGHT - BAR_H, COL_WHITE);
  lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  lcdFillRect(20, 230, 80, 40, 0x2104); lcdFillRect(22, 232, 76, 36, getThemeBg()); lcdDrawTextCentered(20, 80, 242, "<", COL_WHITE, 2);
  lcdFillRect(380, 230, 80, 40, 0x2104); lcdFillRect(382, 232, 76, 36, getThemeBg()); lcdDrawTextCentered(380, 80, 242, ">", COL_WHITE, 2);
  drawControlBar();
}
void drawSteeringArrows() {
  lcdFillRect(20, 230, 80, 40, (steerInput == -1) ? COL_YELLOW : 0x2104); lcdFillRect(22, 232, 76, 36, (steerInput == -1) ? 0x8410 : getThemeBg()); lcdDrawTextCentered(20, 80, 242, "<", COL_WHITE, 2);
  lcdFillRect(380, 230, 80, 40, (steerInput == 1) ? COL_YELLOW : 0x2104); lcdFillRect(382, 232, 76, 36, (steerInput == 1) ? 0x8410 : getThemeBg()); lcdDrawTextCentered(380, 80, 242, ">", COL_WHITE, 2);
}
#define RDLG_W 380
#define RDLG_H 200
#define RDLG_X ((LCD_WIDTH - RDLG_W) / 2)
#define RDLG_Y ((BAR_Y - RDLG_H) / 2)
#define RDLG_TITLEBAR_H 30
#define RD_BTN_W 100
#define RD_BTN_H 36
#define RD_BTN_Y (RDLG_Y + 130)
int racingDifficulty = 0;
void drawRacingDifficultyDialog() {
  drawDialogShell(RDLG_X, RDLG_Y, RDLG_W, RDLG_H, "RACING", COL_RED);
  int closeX = RDLG_X + RDLG_W - 20, closeY = RDLG_Y + 6;
  drawQuestionIcon(RDLG_X + 40, RDLG_Y + 70, 20);
  lcdDrawText(RDLG_X + 80, RDLG_Y + 56, "SELECT A DIFFICULTY", COL_BLACK, 1); lcdDrawText(RDLG_X + 80, RDLG_Y + 72, "MODE", COL_BLACK, 1);
  int gap = (RDLG_W - 3 * RD_BTN_W) / 4;
  int x0 = RDLG_X + gap, x1 = RDLG_X + gap * 2 + RD_BTN_W, x2 = RDLG_X + gap * 3 + RD_BTN_W * 2;
  int xs[3] = {x0, x1, x2}; const char* labels[3] = {"EASY", "MEDIUM", "HARD"};
  for (int i = 0; i < 3; i++) {
    bool sel = (racingDifficulty == i);
    lcdFillRect(xs[i], RD_BTN_Y, RD_BTN_W, RD_BTN_H, COL_BLACK); lcdFillRect(xs[i] + 2, RD_BTN_Y + 2, RD_BTN_W - 4, RD_BTN_H - 4, sel ? 0xC618 : COL_WHITE);
    lcdDrawTextCentered(xs[i], RD_BTN_W, RD_BTN_Y + RD_BTN_H/2 - 4, labels[i], COL_BLACK, 1);
  }
}
void handleRacingDifficultyTouch(uint16_t tx, uint16_t ty) {
  int closeX = RDLG_X + RDLG_W - 20, closeY = RDLG_Y + 6;
  if (tx >= closeX - 6 && tx < closeX + 20 && ty >= closeY - 6 && ty < closeY + 20) { uiClick(); currentMode = MODE_LAUNCHER; drawLauncher(); return; }
  if (ty < RD_BTN_Y || ty >= RD_BTN_Y + RD_BTN_H) return;
  int gap = (RDLG_W - 3 * RD_BTN_W) / 4;
  int x0 = RDLG_X + gap, x1 = RDLG_X + gap * 2 + RD_BTN_W, x2 = RDLG_X + gap * 3 + RD_BTN_W * 2;
  if (tx >= x0 && tx < x0 + RD_BTN_W) { uiClick(); racingDifficulty = 0; currentMode = MODE_RACING; initRacing(); }
  else if (tx >= x1 && tx < x1 + RD_BTN_W) { uiClick(); racingDifficulty = 1; currentMode = MODE_RACING; initRacing(); }
  else if (tx >= x2 && tx < x2 + RD_BTN_W) { uiClick(); racingDifficulty = 2; currentMode = MODE_RACING; initRacing(); }
}
void initRacing() {
  racingCarX = LCD_WIDTH / 2; racingScore = 0;
  { const int startSpeeds[3] = {4, 6, 8}; racingSpeed = startSpeeds[racingDifficulty]; }
  racingGameOver = false; racingRoadOffset = 0; racingObstacleY = -50;
  racingObstacleX = RACING_ROAD_X + random(RACING_ROAD_WIDTH - RACING_OBSTACLE_WIDTH);
  racingLastUpdate = millis(); steerInput = 0; drawRacingBackground();
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + racingRoadOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, COL_YELLOW); }
  int carY = LCD_HEIGHT - BAR_H - RACING_CAR_HEIGHT - 20;
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2, carY, RACING_CAR_WIDTH, RACING_CAR_HEIGHT, COL_RED);
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2 + 5, carY - 5, RACING_CAR_WIDTH - 10, 8, COL_YELLOW);
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2 + 2, carY + RACING_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  lcdFillRect(racingCarX + RACING_CAR_WIDTH/2 - 10, carY + RACING_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  char scoreStr[20]; snprintf(scoreStr, sizeof(scoreStr), "SCORE: %d", racingScore); lcdDrawText(10, 10, scoreStr, COL_WHITE, 1);
  char hiTopStr[20]; snprintf(hiTopStr, sizeof(hiTopStr), "HI: %d", racingHighScore); lcdDrawTextCentered(LCD_WIDTH/2 - 40, 80, 10, hiTopStr, COL_MINT, 1);
  char speedStr[20]; snprintf(speedStr, sizeof(speedStr), "SPEED: %d", racingSpeed * 20); lcdDrawText(LCD_WIDTH - 100, 10, speedStr, COL_WHITE, 1);
  if (imuAvailable) { char gyroStr[20]; snprintf(gyroStr, sizeof(gyroStr), "TILT: %d", accelX / 100); lcdDrawText(LCD_WIDTH / 2 - 30, 20, gyroStr, COL_YELLOW, 1); }
  else { lcdDrawText(LCD_WIDTH / 2 - 40, 20, "NO GYRO", COL_RED, 1); }
}
void updateRacing() {
  if (racingGameOver) return;
  unsigned long now = millis(); if (now - racingLastUpdate < 50) return; racingLastUpdate = now;
  static unsigned long racingLastEngine = 0;
  if (now - racingLastEngine > 180) { racingLastEngine = now; playEngine(racingSpeed); }
  int prevCarX = racingCarX; int prevObsX = racingObstacleX; int prevObsY = racingObstacleY;
  int prevOffset = racingRoadOffset; int prevScore = racingScore; int prevSpeed = racingSpeed;
  imuReadAccel();
  int moveAmount = steerInput * 6;
  if (steerInput == 0 && imuAvailable) {
    int slightTilt = 2000, hardTilt = 4000;
    if (accelX < -hardTilt) moveAmount = -8; else if (accelX < -slightTilt) moveAmount = -4;
    else if (accelX > hardTilt) moveAmount = 8; else if (accelX > slightTilt) moveAmount = 4;
  }
  racingCarX += moveAmount;
  int minX = RACING_ROAD_X + RACING_CAR_WIDTH/2 + 5; int maxX = RACING_ROAD_X + RACING_ROAD_WIDTH - RACING_CAR_WIDTH/2 - 5;
  if (racingCarX < minX) racingCarX = minX; if (racingCarX > maxX) racingCarX = maxX;
  racingRoadOffset = (racingRoadOffset + racingSpeed) % 40; racingObstacleY += racingSpeed;
  int carY = LCD_HEIGHT - BAR_H - RACING_CAR_HEIGHT - 20;
  if (racingObstacleY + RACING_OBSTACLE_HEIGHT > carY && racingObstacleY < carY + RACING_CAR_HEIGHT &&
      racingObstacleX + RACING_OBSTACLE_WIDTH > racingCarX - RACING_CAR_WIDTH/2 && racingObstacleX < racingCarX + RACING_CAR_WIDTH/2) {
    racingGameOver = true; playError();
    if (racingScore > racingHighScore) { racingHighScore = racingScore; saveHighScores(); }
    lcdFillRect(LCD_WIDTH/2 - 90, LCD_HEIGHT/2 - 40, 180, 90, COL_RED);
    lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2 - 30, "GAME OVER!", COL_WHITE, 2);
    char hiStr[24]; snprintf(hiStr, sizeof(hiStr), "HIGH SCORE: %d", racingHighScore);
    lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2, hiStr, COL_YELLOW, 1);
    lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2 + 20, "TAP TO RESTART", COL_WHITE, 1); return;
  }
  if (racingObstacleY > LCD_HEIGHT - BAR_H) {
    racingObstacleY = -50; racingObstacleX = RACING_ROAD_X + random(RACING_ROAD_WIDTH - RACING_OBSTACLE_WIDTH); racingScore += 10;
    { const int levelUpEvery[3] = {30, 20, 10}; const int maxSpeed[3] = {9, 12, 16};
      if (racingScore % levelUpEvery[racingDifficulty] == 0) racingSpeed = min(maxSpeed[racingDifficulty], racingSpeed + 1); }
  }
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + prevOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, 0x4208); }
  if (prevObsY >= 0 && prevObsY < LCD_HEIGHT - BAR_H) lcdFillRect(prevObsX, prevObsY, RACING_OBSTACLE_WIDTH, RACING_OBSTACLE_HEIGHT, 0x4208);
  lcdFillRect(prevCarX - RACING_CAR_WIDTH/2, carY, RACING_CAR_WIDTH, RACING_CAR_HEIGHT, 0x4208);
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + racingRoadOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, COL_YELLOW); }
  if (racingObstacleY >= 0 && racingObstacleY < LCD_HEIGHT - BAR_H) {
    lcdFillRect(racingObstacleX, racingObstacleY, RACING_OBSTACLE_WIDTH, RACING_OBSTACLE_HEIGHT, COL_BLUE);
    lcdFillRect(racingObstacleX + 5, racingObstacleY + 5, RACING_OBSTACLE_WIDTH - 10, 8, COL_GRAY);
    lcdFillRect(racingObstacleX + 2, racingObstacleY + RACING_OBSTACLE_HEIGHT - 10, 8, 10, COL_BLACK);
    lcdFillRect(racingObstacleX + RACING_OBSTACLE_WIDTH - 10, racingObstacleY + RACING_OBSTACLE_HEIGHT - 10, 8, 10, COL_BLACK);
  }
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2, carY, RACING_CAR_WIDTH, RACING_CAR_HEIGHT, COL_RED);
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2 + 5, carY - 5, RACING_CAR_WIDTH - 10, 8, COL_YELLOW);
  lcdFillRect(racingCarX - RACING_CAR_WIDTH/2 + 2, carY + RACING_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  lcdFillRect(racingCarX + RACING_CAR_WIDTH/2 - 10, carY + RACING_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  if (racingScore != prevScore || racingSpeed != prevSpeed) {
    lcdFillRect(10, 10, 100, 10, getThemePrimary()); char scoreStr[20]; snprintf(scoreStr, sizeof(scoreStr), "SCORE: %d", racingScore); lcdDrawText(10, 10, scoreStr, COL_WHITE, 1);
    lcdFillRect(LCD_WIDTH - 100, 10, 100, 10, getThemePrimary()); char speedStr[20]; snprintf(speedStr, sizeof(speedStr), "SPEED: %d", racingSpeed * 20); lcdDrawText(LCD_WIDTH - 100, 10, speedStr, COL_WHITE, 1);
  }
}
void handleRacingTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  if (racingGameOver) { uiClick(); initRacing(); return; }
  if (tx >= 20 && tx <= 100 && ty >= 230 && ty <= 270) { if (steerInput != -1) { steerInput = -1; drawSteeringArrows(); } return; }
  if (tx >= 380 && tx <= 460 && ty >= 230 && ty <= 270) { if (steerInput != 1) { steerInput = 1; drawSteeringArrows(); } return; }
}

// ---------- Space Fighters (Shooter) ----------
#define SF_SHIP_W 30
#define SF_SHIP_H 22
#define SF_ENEMY_W 24
#define SF_ENEMY_H 18
#define SF_BULLET_W 4
#define SF_BULLET_H 10
#define SF_MAX_ENEMIES 4
#define SF_MAX_BULLETS 3
#define SF_MAX_EBULLETS 3
#define SF_ESCAPE_LIMIT 10
struct SFEnemy { int x, y; bool active; int dx; };
struct SFBullet { int x, y; bool active; };
int sfShipX;
int sfScore = 0;
int sfLives = 3;
int sfEscapes = 0;
int sfSteer = 0;
unsigned long sfLastUpdate = 0;
unsigned long sfLastShot = 0;
unsigned long sfLastSpawn = 0;
unsigned long sfLastEnemyShot = 0;
int sfSpeed = 3;
SFEnemy sfEnemies[SF_MAX_ENEMIES];
SFBullet sfBullets[SF_MAX_BULLETS];
SFBullet sfEBullets[SF_MAX_EBULLETS];

int sfShipY() { return LCD_HEIGHT - BAR_H - SF_SHIP_H - 20; }

// ---------- Space Fighters: Difficulty Select ----------
#define SFDLG_W 320
#define SFDLG_H 180
#define SFDLG_X ((LCD_WIDTH - SFDLG_W) / 2)
#define SFDLG_Y ((BAR_Y - SFDLG_H) / 2)
#define SFDLG_TITLEBAR_H 30
#define SFD_BTN_W 100
#define SFD_BTN_H 36
#define SFD_BTN_Y (SFDLG_Y + 130)
void drawSFDifficultyDialog() {
  drawDialogShell(SFDLG_X, SFDLG_Y, SFDLG_W, SFDLG_H, "SPACE FIGHTERS", COL_RED);
  int closeX = SFDLG_X + SFDLG_W - 20, closeY = SFDLG_Y + 6;
  drawQuestionIcon(SFDLG_X + 40, SFDLG_Y + 70, 20);
  lcdDrawText(SFDLG_X + 80, SFDLG_Y + 56, "SELECT A DIFFICULTY", COL_BLACK, 1); lcdDrawText(SFDLG_X + 80, SFDLG_Y + 72, "MODE", COL_BLACK, 1);
  int gap = (SFDLG_W - 3 * SFD_BTN_W) / 4;
  int x0 = SFDLG_X + gap, x1 = SFDLG_X + gap * 2 + SFD_BTN_W, x2 = SFDLG_X + gap * 3 + SFD_BTN_W * 2;
  int xs[3] = {x0, x1, x2}; const char* labels[3] = {"EASY", "MEDIUM", "HARD"};
  for (int i = 0; i < 3; i++) {
    bool sel = (sfDifficulty == i);
    lcdFillRect(xs[i], SFD_BTN_Y, SFD_BTN_W, SFD_BTN_H, COL_BLACK); lcdFillRect(xs[i] + 2, SFD_BTN_Y + 2, SFD_BTN_W - 4, SFD_BTN_H - 4, sel ? 0xC618 : COL_WHITE);
    lcdDrawTextCentered(xs[i], SFD_BTN_W, SFD_BTN_Y + SFD_BTN_H/2 - 4, labels[i], COL_BLACK, 1);
  }
}
void handleSFDifficultyTouch(uint16_t tx, uint16_t ty) {
  int closeX = SFDLG_X + SFDLG_W - 20, closeY = SFDLG_Y + 6;
  if (tx >= closeX - 6 && tx < closeX + 20 && ty >= closeY - 6 && ty < closeY + 20) { uiClick(); currentMode = MODE_LAUNCHER; drawLauncher(); return; }
  if (ty < SFD_BTN_Y || ty >= SFD_BTN_Y + SFD_BTN_H) return;
  int gap = (SFDLG_W - 3 * SFD_BTN_W) / 4;
  int x0 = SFDLG_X + gap, x1 = SFDLG_X + gap * 2 + SFD_BTN_W, x2 = SFDLG_X + gap * 3 + SFD_BTN_W * 2;
  if (tx >= x0 && tx < x0 + SFD_BTN_W) { uiClick(); sfDifficulty = 0; initSpaceFighters(); }
  else if (tx >= x1 && tx < x1 + SFD_BTN_W) { uiClick(); sfDifficulty = 1; initSpaceFighters(); }
  else if (tx >= x2 && tx < x2 + SFD_BTN_W) { uiClick(); sfDifficulty = 2; initSpaceFighters(); }
}

// Player fighter: nose, hull, swept wings, cockpit
// Small red heart icon used for the lives HUD
void drawSFHeart(int cx, int cy, uint16_t color) {
  lcdFillCircle(cx - 4, cy - 2, 4, color);
  lcdFillCircle(cx + 4, cy - 2, 4, color);
  int h = 8;
  for (int yy = 0; yy <= h; yy++) {
    int w = 10 * (h - yy) / h;
    lcdFillRect(cx - w / 2, cy - 2 + yy, w, 1, color);
  }
}
void drawSFShip(uint16_t color) {
  int y = sfShipY();
  lcdFillTriangle(sfShipX, y, y + 14, 6, color);
  lcdFillRect(sfShipX - 6, y + 12, 12, 10, color);
  lcdFillRect(sfShipX - 15, y + 15, 9, 6, color);
  lcdFillRect(sfShipX + 6, y + 15, 9, 6, color);
  lcdFillRect(sfShipX - 3, y + 20, 6, 4, color);
  lcdFillCircle(sfShipX, y + 9, 3, (color == COL_BLACK) ? COL_BLACK : COL_MINT);
  // Damage cracks on the hull, scaled to hits taken (3 lives = clean, 1 life = heavily cracked)
  if (color != COL_BLACK) {
    int hits = 3 - sfLives;
    if (hits >= 1) { // small, noticeable cracks
      lcdDrawLine(sfShipX - 5, y + 13, sfShipX - 2, y + 18, COL_BLACK);
      lcdDrawLine(sfShipX + 3, y + 14, sfShipX + 6, y + 19, COL_BLACK);
    }
    if (hits >= 2) { // large, noticeable cracks + scorch marks
      lcdDrawLine(sfShipX - 5, y + 13, sfShipX - 2, y + 18, COL_BLACK);
      lcdDrawLine(sfShipX - 2, y + 18, sfShipX - 6, y + 21, COL_BLACK);
      lcdDrawLine(sfShipX + 3, y + 14, sfShipX + 6, y + 19, COL_BLACK);
      lcdDrawLine(sfShipX + 6, y + 19, sfShipX + 2, y + 22, COL_BLACK);
      lcdDrawLine(sfShipX - 1, y + 12, sfShipX + 1, y + 20, COL_BLACK);
      lcdFillCircle(sfShipX - 8, y + 17, 2, 0x2104);
      lcdFillCircle(sfShipX + 8, y + 17, 2, 0x2104);
    }
  }
}
// Enemy fighter: saucer body, side fins, dark cockpit
void drawSFEnemy(int idx, uint16_t color) {
  int x = sfEnemies[idx].x, y = sfEnemies[idx].y;
  lcdFillRect(x + 5, y, SF_ENEMY_W - 10, 6, color);
  lcdFillRect(x, y + 6, SF_ENEMY_W, 8, color);
  lcdFillRect(x + 1, y + 14, 6, 4, color);
  lcdFillRect(x + SF_ENEMY_W - 7, y + 14, 6, 4, color);
  lcdFillCircle(x + SF_ENEMY_W/2, y + 9, 3, (color == COL_BLACK) ? COL_BLACK : COL_BLACK);
}
void drawSFArrows() {
  lcdFillRect(20, 230, 80, 40, (sfSteer == -1) ? COL_YELLOW : 0x2104); lcdFillRect(22, 232, 76, 36, (sfSteer == -1) ? 0x8410 : COL_BLACK); lcdDrawTextCentered(20, 80, 242, "<", COL_WHITE, 2);
  lcdFillRect(380, 230, 80, 40, (sfSteer == 1) ? COL_YELLOW : 0x2104); lcdFillRect(382, 232, 76, 36, (sfSteer == 1) ? 0x8410 : COL_BLACK); lcdDrawTextCentered(380, 80, 242, ">", COL_WHITE, 2);
}
void drawSFHud() {
  lcdFillRect(0, 0, LCD_WIDTH, 20, COL_BLACK);
  char s[24]; snprintf(s, sizeof(s), "SCORE: %d", sfScore); lcdDrawText(10, 6, s, COL_WHITE, 1);
  lcdDrawTextCentered(0, LCD_WIDTH, 6, "SPACE FIGHTERS", COL_MINT, 1);
  char hi[16]; snprintf(hi, sizeof(hi), "HI: %d", sfHighScore); lcdDrawText(LCD_WIDTH - 170, 6, hi, 0xFFE0, 1);
  for (int i = 0; i < 3; i++) drawSFHeart(LCD_WIDTH - 58 + i * 18, 10, (i < sfLives) ? COL_RED : 0x8410);
}
void sfSpawnEnemy() {
  for (int i = 0; i < SF_MAX_ENEMIES; i++) {
    if (!sfEnemies[i].active) {
      sfEnemies[i].active = true;
      sfEnemies[i].x = random(LCD_WIDTH - SF_ENEMY_W);
      sfEnemies[i].y = -SF_ENEMY_H;
      sfEnemies[i].dx = (random(2) == 0) ? 1 : -1;
      return;
    }
  }
}
void sfShowGameOver() {
  lcdFillScreen(COL_BLACK);
  lcdDrawTextCentered(0, LCD_WIDTH, 100, "GAME OVER", COL_RED, 3);
  char s[24]; snprintf(s, sizeof(s), "SCORE: %d", sfScore); lcdDrawTextCentered(0, LCD_WIDTH, 150, s, COL_WHITE, 2);
  char hi[24]; snprintf(hi, sizeof(hi), "HIGH SCORE: %d", sfHighScore); lcdDrawTextCentered(0, LCD_WIDTH, 180, hi, COL_YELLOW, 1);
  lcdDrawTextCentered(0, LCD_WIDTH, 210, "TAP TO RESTART", COL_MINT, 1);
}
// Ship destruction: expanding fireball + debris, played once on the fatal hit
void sfDrawExplosion() {
  int cx = sfShipX, cy = sfShipY() + 10;
  lcdFillCircle(cx, cy, 8, COL_WHITE); delay(70);
  lcdFillCircle(cx, cy, 14, COL_YELLOW); delay(70);
  lcdFillCircle(cx, cy, 20, 0xFD20); delay(70);
  lcdFillCircle(cx, cy, 26, COL_RED); delay(90);
  for (int i = 0; i < 14; i++) lcdFillRect(cx - 26 + random(52), cy - 26 + random(52), 2, 2, i % 2 ? COL_YELLOW : COL_GRAY);
  delay(120);
  lcdFillCircle(cx, cy, 30, COL_BLACK);
}
// Lose a life: show the critical error dialog, then either resume or end the game
void sfLoseLife(const char* line1, const char* line2) {
  sfLives--;
  playError();
  if (sfLives <= 0) {
    sfGameOver = true;
    sfDrawExplosion();
    if (sfScore > sfHighScore) { sfHighScore = sfScore; saveHighScores(); }
  }
  showMessageBox(MSG_WARN, "SPACE FIGHTERS", line1, line2, MODE_SPACE_FIGHTERS);
}
void sfRedrawScreen() {
  lcdFillScreen(COL_BLACK);
  for (int i = 0; i < 40; i++) lcdFillRect(random(LCD_WIDTH), random(LCD_HEIGHT - BAR_H), 1, 1, COL_WHITE);
  drawSFHud(); drawSFArrows(); drawSFShip(COL_GREEN);
  for (int i = 0; i < SF_MAX_ENEMIES; i++) if (sfEnemies[i].active) drawSFEnemy(i, COL_RED);
  for (int i = 0; i < SF_MAX_BULLETS; i++) if (sfBullets[i].active) lcdFillRect(sfBullets[i].x, sfBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_YELLOW);
  for (int i = 0; i < SF_MAX_EBULLETS; i++) if (sfEBullets[i].active) lcdFillRect(sfEBullets[i].x, sfEBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_RED);
  drawControlBar();
}
void initSpaceFighters() {
  currentMode = MODE_SPACE_FIGHTERS;
  sfShipX = LCD_WIDTH / 2; sfScore = 0; sfLives = 3; sfEscapes = 0; sfGameOver = false; sfSteer = 0; sfSpeed = 3;
  memset(sfEnemies, 0, sizeof(sfEnemies)); memset(sfBullets, 0, sizeof(sfBullets)); memset(sfEBullets, 0, sizeof(sfEBullets));
  sfLastUpdate = millis(); sfLastShot = millis(); sfLastSpawn = millis(); sfLastEnemyShot = millis();
  lcdFillScreen(COL_BLACK);
  for (int i = 0; i < 40; i++) lcdFillRect(random(LCD_WIDTH), random(LCD_HEIGHT - BAR_H), 1, 1, COL_WHITE);
  drawSFHud(); drawSFArrows(); drawSFShip(COL_GREEN); drawControlBar();
}
void updateSpaceFighters() {
  if (sfGameOver) return;
  unsigned long now = millis();
  if (now - sfLastUpdate < 40) return;
  sfLastUpdate = now;

  drawSFShip(COL_BLACK);
  int moveAmount = sfSteer * 7;
  sfShipX += moveAmount;
  int minX = SF_SHIP_W/2 + 2, maxX = LCD_WIDTH - SF_SHIP_W/2 - 2;
  if (sfShipX < minX) sfShipX = minX; if (sfShipX > maxX) sfShipX = maxX;
  drawSFShip(COL_GREEN);

  if (now - sfLastShot > 350) {
    sfLastShot = now;
    for (int i = 0; i < SF_MAX_BULLETS; i++) {
      if (!sfBullets[i].active) { sfBullets[i].active = true; sfBullets[i].x = sfShipX - SF_BULLET_W/2; sfBullets[i].y = sfShipY() - SF_BULLET_H; playLaser(); break; }
    }
  }

  if (now - sfLastSpawn > 900) {
    sfLastSpawn = now; sfSpawnEnemy();
    if (sfSpeed < 7) sfSpeed++;
  }

  // Enemies return fire (HARD only: EASY/MEDIUM have no enemy shots)
  if (sfDifficulty == 2 && now - sfLastEnemyShot > 1100) {
    sfLastEnemyShot = now;
    int candidates[SF_MAX_ENEMIES]; int nCand = 0;
    for (int i = 0; i < SF_MAX_ENEMIES; i++) if (sfEnemies[i].active) candidates[nCand++] = i;
    if (nCand > 0) {
      int pick = candidates[random(nCand)];
      for (int i = 0; i < SF_MAX_EBULLETS; i++) {
        if (!sfEBullets[i].active) { sfEBullets[i].active = true; sfEBullets[i].x = sfEnemies[pick].x + SF_ENEMY_W/2 - SF_BULLET_W/2; sfEBullets[i].y = sfEnemies[pick].y + SF_ENEMY_H; break; }
      }
    }
  }

  for (int i = 0; i < SF_MAX_BULLETS; i++) {
    if (sfBullets[i].active) {
      lcdFillRect(sfBullets[i].x, sfBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_BLACK);
      sfBullets[i].y -= 10;
      if (sfBullets[i].y < 20) { sfBullets[i].active = false; continue; }
      lcdFillRect(sfBullets[i].x, sfBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_YELLOW);
    }
  }

  int shipY = sfShipY();

  // Move enemy bullets, check hits on player
  for (int i = 0; i < SF_MAX_EBULLETS; i++) {
    if (!sfEBullets[i].active) continue;
    lcdFillRect(sfEBullets[i].x, sfEBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_BLACK);
    sfEBullets[i].y += 8;
    if (sfEBullets[i].y > LCD_HEIGHT - BAR_H) { sfEBullets[i].active = false; continue; }
    if (sfEBullets[i].y + SF_BULLET_H > shipY && sfEBullets[i].y < shipY + SF_SHIP_H &&
        sfEBullets[i].x + SF_BULLET_W > sfShipX - SF_SHIP_W/2 && sfEBullets[i].x < sfShipX + SF_SHIP_W/2) {
      sfEBullets[i].active = false;
      sfLoseLife("HIT BY ENEMY", "LASER FIRE!");
      return;
    }
    lcdFillRect(sfEBullets[i].x, sfEBullets[i].y, SF_BULLET_W, SF_BULLET_H, COL_RED);
  }

  for (int i = 0; i < SF_MAX_ENEMIES; i++) {
    if (!sfEnemies[i].active) continue;
    drawSFEnemy(i, COL_BLACK);
    sfEnemies[i].y += sfSpeed / 2 + 2;
    sfEnemies[i].x += sfEnemies[i].dx;
    if (sfEnemies[i].x <= 0 || sfEnemies[i].x >= LCD_WIDTH - SF_ENEMY_W) sfEnemies[i].dx = -sfEnemies[i].dx;
    if (sfEnemies[i].y > LCD_HEIGHT - BAR_H) {
      sfEnemies[i].active = false;
      sfEscapes++;
      if (sfEscapes > SF_ESCAPE_LIMIT) {
        sfEscapes = 0;
        if (sfDifficulty >= 1) { // MEDIUM/HARD: taskbar can be eaten; EASY: no penalty
          sfLoseLife("THE TASKBAR WAS", "EATEN!");
          return;
        }
      }
      continue;
    }

    if (sfEnemies[i].y + SF_ENEMY_H > shipY && sfEnemies[i].y < shipY + SF_SHIP_H &&
        sfEnemies[i].x + SF_ENEMY_W > sfShipX - SF_SHIP_W/2 && sfEnemies[i].x < sfShipX + SF_SHIP_W/2) {
      sfEnemies[i].active = false;
      sfLoseLife("ENEMY SHIP", "COLLISION!");
      return;
    }
    drawSFEnemy(i, COL_RED);
  }

  for (int b = 0; b < SF_MAX_BULLETS; b++) {
    if (!sfBullets[b].active) continue;
    for (int i = 0; i < SF_MAX_ENEMIES; i++) {
      if (!sfEnemies[i].active) continue;
      if (sfBullets[b].x < sfEnemies[i].x + SF_ENEMY_W && sfBullets[b].x + SF_BULLET_W > sfEnemies[i].x &&
          sfBullets[b].y < sfEnemies[i].y + SF_ENEMY_H && sfBullets[b].y + SF_BULLET_H > sfEnemies[i].y) {
        lcdFillRect(sfBullets[b].x, sfBullets[b].y, SF_BULLET_W, SF_BULLET_H, COL_BLACK);
        drawSFEnemy(i, COL_BLACK);
        sfBullets[b].active = false; sfEnemies[i].active = false;
        sfScore += 10;
        uiClick();
        lcdFillRect(0, 0, 120, 16, COL_BLACK);
        char s[24]; snprintf(s, sizeof(s), "SCORE: %d", sfScore); lcdDrawText(10, 6, s, COL_WHITE, 1);
        break;
      }
    }
  }
}
void handleSpaceFightersTouch(uint16_t tx, uint16_t ty) {
  if (sfGameOver) { uiClick(); initSpaceFighters(); return; }
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  if (tx >= 20 && tx <= 100 && ty >= 230 && ty <= 270) { if (sfSteer != -1) { sfSteer = -1; drawSFArrows(); } return; }
  if (tx >= 380 && tx <= 460 && ty >= 230 && ty <= 270) { if (sfSteer != 1) { sfSteer = 1; drawSFArrows(); } return; }
}

// ---------- Clock App (NTP-based) ----------
char clkLastStr[24] = "";
bool clkSynced = false;
void ntpSyncNow() { configTime(settingTimezoneOffset * 3600, 0, "pool.ntp.org", "time.nist.gov"); }

// ---------- Notification System ----------
char notificationText[64] = "";
unsigned long notificationStartTime = 0;
int notificationDuration = 3000; // 3 seconds (no VIEW button)
int notificationDurationWithView = 6000; // longer when there's an action to take
bool notificationActive = false;
bool notificationHasView = false;
AppMode notificationViewMode = MODE_LAUNCHER;
#define NOTIF_BAR_H 26
#define NOTIF_VIEW_BTN_W 56

void showNotification(const char* text) {
  strncpy(notificationText, text, sizeof(notificationText)-1);
  notificationText[sizeof(notificationText)-1] = 0;
  notificationStartTime = millis();
  notificationActive = true;
  notificationHasView = false;
}

// Same as showNotification(), but adds a tappable "VIEW" button that switches
// currentMode to viewMode when tapped (e.g. jump back into a finished scan).
void showNotificationWithView(const char* text, AppMode viewMode) {
  strncpy(notificationText, text, sizeof(notificationText)-1);
  notificationText[sizeof(notificationText)-1] = 0;
  notificationStartTime = millis();
  notificationActive = true;
  notificationHasView = true;
  notificationViewMode = viewMode;
}

void updateNotification() {
  if (notificationActive) {
    int dur = notificationHasView ? notificationDurationWithView : notificationDuration;
    if (millis() - notificationStartTime > (unsigned long)dur) {
      notificationText[0] = 0;
      notificationActive = false;
      notificationHasView = false;
    }
  }
}

void drawNotification() {
  if (notificationActive && notificationText[0] != 0) {
    lcdFillRect(0, 0, LCD_WIDTH, NOTIF_BAR_H, COL_BLUE);
    lcdFillRect(0, NOTIF_BAR_H - 2, LCD_WIDTH, 2, COL_MINT_DK);
    int textW = notificationHasView ? LCD_WIDTH - NOTIF_VIEW_BTN_W : LCD_WIDTH;
    lcdDrawTextCentered(0, textW, 6, notificationText, COL_WHITE, 1);
    if (notificationHasView) {
      lcdFillRect(LCD_WIDTH - NOTIF_VIEW_BTN_W, 2, NOTIF_VIEW_BTN_W - 2, NOTIF_BAR_H - 4, COL_GREEN);
      lcdDrawTextCentered(LCD_WIDTH - NOTIF_VIEW_BTN_W, NOTIF_VIEW_BTN_W - 2, 6, "VIEW", COL_WHITE, 1);
    }
  }
}

// Handles a tap on the notification banner (VIEW button, or tapping the banner
// to dismiss it). Returns true if the tap was consumed by the banner.
bool handleNotificationTouch(uint16_t tx, uint16_t ty) {
  if (!notificationActive || ty >= NOTIF_BAR_H) return false;
  uiClick();
  if (notificationHasView && tx >= LCD_WIDTH - NOTIF_VIEW_BTN_W) {
    AppMode target = notificationViewMode;
    notificationActive = false;
    notificationHasView = false;
    notificationText[0] = 0;
    currentMode = target;
    GpuTaskMessage msg = {GPU_CMD_NONE};
    switch (target) {
      case MODE_LAUNCHER: msg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
      case MODE_IP_EXPLORER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawIpExplorer; break;
      case MODE_EMAIL_INBOX: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawEmailInbox; break;
      case MODE_TASK_MANAGER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawTaskManager; break;
      case MODE_FILE_BROWSER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawFileBrowser; break;
      default: msg.cmd = GPU_CMD_DRAW_LAUNCHER; currentMode = MODE_LAUNCHER; break;
    }
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  } else {
    // Tapped the banner but not VIEW - just dismiss it and redraw what's underneath
    notificationActive = false;
    notificationHasView = false;
    notificationText[0] = 0;
    GpuTaskMessage msg = {GPU_CMD_NONE};
    switch (currentMode) {
      case MODE_LAUNCHER: msg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
      case MODE_IP_EXPLORER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawIpExplorer; break;
      case MODE_EMAIL_INBOX: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawEmailInbox; break;
      case MODE_TASK_MANAGER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawTaskManager; break;
      case MODE_FILE_BROWSER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawFileBrowser; break;
      default: msg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
    }
    if (msg.cmd != GPU_CMD_NONE) xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  }
  return true;
}

// ---------- Real SMTP with WiFiClientSecure for Gmail ----------
// Reads one full SMTP reply, which may span several lines (e.g.
// "250-...", "250-...", "250 ..."). Continuation lines start with
// "code-", the final line starts with "code ". Returns the text of the
// FINAL line (the one callers should check the status code against) and
// makes sure every continuation line is drained from the socket first,
// so a leftover line can never be mistaken for the reply to the next
// command. Returns "" on timeout.
String readSmtpResponse(WiFiClientSecure &client, unsigned long timeoutMs) {
  String finalLine = "";
  unsigned long start = millis();
  bool sawAnyLine = false;
  while (millis() - start < timeoutMs) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim(); // drop trailing \r and any stray whitespace
      if (line.length() == 0) continue; // ignore stray blank lines
      Serial.println(line);
      sawAnyLine = true;
      start = millis(); // reset timeout while more lines keep arriving
      finalLine = line;
      // Final line has a space (not '-') as the 4th character, e.g. "250 OK"
      if (line.length() < 4 || line.charAt(3) != '-') break;
    }
  }
  if (!sawAnyLine) return "";
  return finalLine;
}

bool sendEmail(const char* to, const char* subject, const char* body) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    showNotification("WiFi not connected");
    return false;
  }
  
  if (!gmailConfigured || strlen(gmailUsername) == 0 || strlen(gmailPassword) == 0) {
    Serial.println("Gmail not configured");
    showNotification("Gmail not configured");
    return false;
  }
  
  WiFiClientSecure client;
  const char* smtpServer = "smtp.gmail.com";
  const int smtpPort = 465; // SSL port
  
  Serial.println("Connecting to SMTP server with SSL...");
  
  // Set SSL/TLS configuration
  client.setInsecure(); // Skip certificate validation for self-signed certs
  
  if (!client.connect(smtpServer, smtpPort)) {
    Serial.println("SMTP SSL connection failed");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "COULD NOT CONNECT", "TO GMAIL", MODE_EMAIL_COMPOSE);
    return false;
  }
  
  Serial.println("Connected to SMTP server with SSL");
  
  // Wait for server greeting
  String response = readSmtpResponse(client, 10000);
  if (!response.startsWith("220")) {
    Serial.println("No 220 greeting from server");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "SERVER ERROR:", "NO GREETING", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // EHLO
  client.println("EHLO esp32");
  response = readSmtpResponse(client, 5000);
  if (!response.startsWith("250")) {
    Serial.println("EHLO failed or timed out");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "SERVER DID NOT", "ACCEPT EHLO", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // AUTH LOGIN
  client.println("AUTH LOGIN");
  response = readSmtpResponse(client, 5000);
  if (!response.startsWith("334")) {
    Serial.println("AUTH LOGIN not accepted or timed out");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "SERVER REJECTED", "AUTH LOGIN", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // Send username (base64 encoded)
  String usernameBase64 = base64Encode(gmailUsername);
  client.println(usernameBase64);
  response = readSmtpResponse(client, 5000);
  if (response.startsWith("535")) {
    Serial.println("Authentication failed");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "AUTH FAILED: USE A", "GMAIL APP PASSWORD", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  if (!response.startsWith("334")) {
    Serial.println("Username step not accepted or timed out");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "SERVER REJECTED", "USERNAME", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // Send password (base64 encoded)
  String passwordBase64 = base64Encode(gmailPassword);
  client.println(passwordBase64);
  response = readSmtpResponse(client, 5000);
  if (response.startsWith("535")) {
    Serial.println("Authentication failed");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "AUTH FAILED: USE A", "GMAIL APP PASSWORD", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  if (!response.startsWith("235")) {
    Serial.println("Authentication not confirmed or timed out");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "AUTH TIMED OUT:", "CHECK WIFI/RETRY", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // MAIL FROM
  client.print("MAIL FROM:<");
  client.print(gmailUsername);
  client.println(">");
  response = readSmtpResponse(client, 5000);
  if (!response.startsWith("250")) {
    Serial.println("Sender rejected");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "SENDER ADDRESS", "REJECTED", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // RCPT TO
  client.print("RCPT TO:<");
  client.print(to);
  client.println(">");
  response = readSmtpResponse(client, 5000);
  if (!response.startsWith("250")) {
    Serial.println("Recipient rejected");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "RECIPIENT", "REJECTED", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // DATA
  client.println("DATA");
  response = readSmtpResponse(client, 5000);
  if (!response.startsWith("354")) {
    Serial.println("DATA command not accepted or timed out");
    showMessageBox(MSG_CRIT, "EMAIL ERROR", "SERVER REJECTED", "DATA COMMAND", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // Email headers and body
  client.print("From: ");
  client.println(gmailUsername);
  client.print("To: ");
  client.println(to);
  client.print("Subject: ");
  client.println(subject);
  client.println("MIME-Version: 1.0");
  client.println("Content-Type: text/plain; charset=UTF-8");
  client.println();
  client.println(body);
  client.println(".");
  
  response = readSmtpResponse(client, 10000);
  if (!response.startsWith("250")) {
    Serial.println("Email rejected by server");
    showMessageBox(MSG_WARN, "EMAIL ERROR", "REJECTED BY", "SERVER", MODE_EMAIL_COMPOSE);
    client.stop();
    return false;
  }
  
  // QUIT
  client.println("QUIT");
  client.stop();
  
  Serial.println("Email sent successfully");
  return true;
}

// Simple base64 encoding function
String base64Encode(const char* input) {
  const char* base64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String result = "";
  int i = 0;
  int len = strlen(input);
  
  while (i < len) {
    unsigned char a = input[i++];
    unsigned char b = (i < len) ? input[i++] : 0;
    unsigned char c = (i < len) ? input[i++] : 0;
    
    unsigned int triple = (a << 16) | (b << 8) | c;
    
    result += base64Chars[(triple >> 18) & 0x3F];
    result += base64Chars[(triple >> 12) & 0x3F];
    result += base64Chars[(triple >> 6) & 0x3F];
    result += base64Chars[triple & 0x3F];
  }
  
  // Add padding
  int padding = len % 3;
  if (padding == 1) {
    result[result.length() - 2] = '=';
    result[result.length() - 1] = '=';
  } else if (padding == 2) {
    result[result.length() - 1] = '=';
  }
  
  return result;
}

// ---------- Email Notification System ----------
#define MAX_EMAILS 20
struct EmailMessage {
  char sender[64];
  char subject[128];
  char body[256];
  char timestamp[32];
  bool unread;
  bool isLocal; // true for locally sent emails
};
EmailMessage emails[MAX_EMAILS];
int emailCount = 0;

// Compose email data
char composeTo[64] = "";
char composeSubject[128] = "";
char composeBody[256] = "";
int composeField = 0; // 0=to, 1=subject, 2=body

unsigned long lastEmailCheck = 0;
int emailCheckInterval = 60000; // Check every 60 seconds
int unreadEmailCount = 0; // Not used since receiving not implemented

// Email receiving not implemented - requires IMAP over SSL
// This is complex to implement without external libraries
// Only sending is available via SMTP

void checkEmails() {
  // Email receiving not implemented - requires IMAP over SSL
  // This is complex to implement without external libraries
  // Only sending is available via SMTP
  return;
}

void drawEmailNotification() {
  // Since we can't receive emails without IMAP SSL libraries,
  // the notification badge shows sent email count instead
  if (emailCount > 0) {
    // Draw notification badge for sent emails (green instead of red)
    int badgeX = BAR_BTN_W * 7 + BAR_BTN_W - 20;
    int badgeY = BAR_Y;
    lcdFillCircle(badgeX, badgeY + 8, 8, COL_GREEN);
    char countStr[4];
    snprintf(countStr, sizeof(countStr), "%d", emailCount);
    lcdDrawTextCentered(badgeX - 8, 16, badgeY + 4, countStr, COL_WHITE, 1);
  }
}

// ---------- Email App ----------
enum EmailFolder { EMAIL_FOLDER_INBOX = 0, EMAIL_FOLDER_SENT = 1, EMAIL_FOLDER_SPAM = 2 };
int currentEmailFolder = EMAIL_FOLDER_SENT; // Only SENT has real data (no IMAP receiving)

#define EMAIL_HEADER_H  36
#define EMAIL_SIDEBAR_W 70
#define EMAIL_SIDEBAR_Y0 EMAIL_HEADER_H
#define EMAIL_SIDEBAR_TAB_H ((BAR_Y - EMAIL_SIDEBAR_Y0) / 3)

// Compose screen: buttons sit just above the control bar, and the keyboard
// is sized to fit entirely between the Body field and the buttons so
// nothing overlaps or gets drawn over.
#define EMAIL_COMPOSE_BTN_H 26
#define EMAIL_COMPOSE_BTN_Y (BAR_Y - EMAIL_COMPOSE_BTN_H - 2)
#define EMAIL_KB_START_Y 150
#define EMAIL_KB_KEY_W 42
#define EMAIL_KB_KEY_H 24
#define EMAIL_KB_START_X 30

void drawEmailInbox() {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, EMAIL_HEADER_H, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  const char* folderTitle = (currentEmailFolder == EMAIL_FOLDER_INBOX) ? "INBOX" :
                             (currentEmailFolder == EMAIL_FOLDER_SPAM) ? "SPAM" : "SENT";
  lcdDrawTextCentered(50, LCD_WIDTH - 130, 10, folderTitle, COL_WHITE, 1);
  
  // Compose button
  lcdFillRect(LCD_WIDTH - 80, 5, 70, 26, COL_GREEN);
  lcdDrawTextCentered(LCD_WIDTH - 80, 70, 13, "COMPOSE", COL_WHITE, 1);
  
  // ---- Sidebar (INBOX / SENT / SPAM) ----
  lcdFillRect(0, EMAIL_SIDEBAR_Y0, EMAIL_SIDEBAR_W, BAR_Y - EMAIL_SIDEBAR_Y0, getThemeBg());
  lcdFillRect(EMAIL_SIDEBAR_W - 2, EMAIL_SIDEBAR_Y0, 2, BAR_Y - EMAIL_SIDEBAR_Y0, getThemePrimary());
  const char* tabLabels[3] = {"INBOX", "SENT", "SPAM"};
  for (int t = 0; t < 3; t++) {
    int tabY = EMAIL_SIDEBAR_Y0 + t * EMAIL_SIDEBAR_TAB_H;
    bool selected = (currentEmailFolder == t);
    lcdFillRect(0, tabY, EMAIL_SIDEBAR_W - 2, EMAIL_SIDEBAR_TAB_H - 1, selected ? getThemeSecondary() : getThemeBg());
    if (selected) lcdFillRect(0, tabY, 3, EMAIL_SIDEBAR_TAB_H - 1, COL_YELLOW);
    lcdDrawTextCentered(0, EMAIL_SIDEBAR_W - 2, tabY + EMAIL_SIDEBAR_TAB_H / 2 - 4, tabLabels[t], COL_WHITE, 1);
  }
  
  // Email list (right of sidebar)
  int listX = EMAIL_SIDEBAR_W + 4;
  int listW = LCD_WIDTH - listX - 4;
  int listY = EMAIL_HEADER_H + 4;
  int rowH = 30;
  
  if (currentEmailFolder == EMAIL_FOLDER_INBOX) {
    lcdDrawTextCentered(listX, listW, listY + 50, "RECEIVING NOT", COL_GRAY, 1);
    lcdDrawTextCentered(listX, listW, listY + 70, "AVAILABLE (NO IMAP)", COL_GRAY, 1);
  } else if (currentEmailFolder == EMAIL_FOLDER_SPAM) {
    lcdDrawTextCentered(listX, listW, listY + 50, "NO SPAM DETECTED", COL_GRAY, 1);
    lcdDrawTextCentered(listX, listW, listY + 70, "(RECEIVING NOT AVAILABLE)", COL_GRAY, 1);
  } else if (emailCount == 0) {
    lcdDrawTextCentered(listX, listW, listY + 50, "No sent emails", COL_GRAY, 1);
    lcdDrawTextCentered(listX, listW, listY + 70, "Use COMPOSE to send", COL_GRAY, 1);
  } else {
    for (int i = 0; i < emailCount && i < 8; i++) {
      int y = listY + i * rowH;
      EmailMessage* email = &emails[i];
      
      // Email row
      lcdFillRect(listX, y, listW, rowH - 2, email->unread ? getThemeSecondary() : getThemeBg());
      
      // Unread indicator
      if (email->unread) {
        lcdFillCircle(listX + 10, y + rowH/2, 4, COL_BLUE);
      }
      
      // Sender
      lcdDrawText(listX + 20, y + 4, email->sender, COL_WHITE, 1);
      
      // Subject (truncated)
      char subjDisp[26];
      strncpy(subjDisp, email->subject, 25);
      subjDisp[25] = 0;
      lcdDrawText(listX + 20, y + 16, subjDisp, COL_GRAY, 1);
      
      // Timestamp
      lcdDrawText(LCD_WIDTH - 60, y + 4, email->timestamp, COL_GRAY, 1);
    }
  }
  
  drawControlBar();
}

void handleEmailInboxTouch(uint16_t tx, uint16_t ty) {
  // Check if touch is in control bar area first
  if (ty >= BAR_Y) {
    handleControlBarTouch(tx, ty);
    return;
  }
  
  // Back button (top-left label only, not the whole header)
  if (tx < 80 && ty < EMAIL_HEADER_H) {
    uiClick();
    currentMode = MODE_LAUNCHER;
    drawLauncher();
    return;
  }
  
  // Compose button
  if (tx >= LCD_WIDTH - 80 && tx < LCD_WIDTH - 10 && ty >= 5 && ty < 31) {
    uiClick();
    composeTo[0] = 0;
    composeSubject[0] = 0;
    composeBody[0] = 0;
    composeField = 0;
    isNumLayout = false; // Reset to alpha layout
    shiftActive = false; // Reset shift
    currentMode = MODE_EMAIL_COMPOSE;
    drawEmailCompose();
    return;
  }
  
  // Sidebar folder tabs
  if (tx < EMAIL_SIDEBAR_W && ty >= EMAIL_SIDEBAR_Y0 && ty < BAR_Y) {
    int tab = (ty - EMAIL_SIDEBAR_Y0) / EMAIL_SIDEBAR_TAB_H;
    if (tab >= 0 && tab < 3 && tab != currentEmailFolder) {
      uiClick();
      currentEmailFolder = tab;
      drawEmailInbox();
    }
    return;
  }
  
  // Email list (SENT folder only has real entries)
  if (currentEmailFolder == EMAIL_FOLDER_SENT) {
    int listY = EMAIL_HEADER_H + 4;
    int rowH = 30;
    if (ty >= listY && ty < listY + 8 * rowH) {
      int idx = (ty - listY) / rowH;
      if (idx >= 0 && idx < emailCount) {
        uiClick();
        // Since we only have sent emails now, just mark as read
        emails[idx].unread = false;
        drawEmailInbox();
      }
    }
  }
}

void drawEmailCompose() {
  lcdFillScreen(getThemeBg());
  
  // Remove top header to save space
  // lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary());
  // lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  // lcdDrawTextCentered(50, LCD_WIDTH - 50, 10, "COMPOSE", COL_WHITE, 1);
  
  // Send button (sits just above the control bar)
  lcdFillRect(LCD_WIDTH - 80, EMAIL_COMPOSE_BTN_Y, 70, EMAIL_COMPOSE_BTN_H, COL_GREEN);
  lcdDrawTextCentered(LCD_WIDTH - 80, 70, EMAIL_COMPOSE_BTN_Y + 4, "SEND", COL_WHITE, 1);
  
  // Back button (sits just above the control bar)
  lcdFillRect(10, EMAIL_COMPOSE_BTN_Y, 70, EMAIL_COMPOSE_BTN_H, COL_RED);
  lcdDrawTextCentered(10, 70, EMAIL_COMPOSE_BTN_Y + 4, "BACK", COL_WHITE, 1);
  
  int fieldY = 30; // Moved up since no header
  
  // To field
  lcdDrawText(10, fieldY, "TO:", COL_WHITE, 1);
  lcdFillRect(40, fieldY - 2, LCD_WIDTH - 130, 24, COL_BLACK);
  lcdDrawText(45, fieldY - 2, composeTo, COL_WHITE, 1);
  if (composeField == 0) lcdDrawRect(39, fieldY - 3, LCD_WIDTH - 128, 26, COL_YELLOW);
  
  // Subject field
  int subjY = fieldY + 35;
  lcdDrawText(10, subjY, "SUBJECT:", COL_WHITE, 1);
  lcdFillRect(80, subjY - 2, LCD_WIDTH - 130, 24, COL_BLACK);
  lcdDrawText(85, subjY - 2, composeSubject, COL_WHITE, 1);
  if (composeField == 1) lcdDrawRect(79, subjY - 3, LCD_WIDTH - 128, 26, COL_YELLOW);
  
  // Body field
  int bodyY = subjY + 35;
  lcdDrawText(10, bodyY, "BODY:", COL_WHITE, 1);
  lcdFillRect(10, bodyY + 20, LCD_WIDTH - 20, 30, COL_BLACK); // Shrunk so it doesn't overlap the keyboard below
  // Show first few chars of body
  char bodyDisp[30];
  strncpy(bodyDisp, composeBody, 29);
  bodyDisp[29] = 0;
  lcdDrawText(15, bodyY + 24, bodyDisp, COL_WHITE, 1);
  if (composeField == 2) lcdDrawRect(9, bodyY + 19, LCD_WIDTH - 18, 32, COL_YELLOW);
  
  // Draw keyboard (sized to fit above the SEND/BACK buttons, no overlap)
  int kbStartY = EMAIL_KB_START_Y;
  int keyW = EMAIL_KB_KEY_W;
  int keyH = EMAIL_KB_KEY_H;
  int startX = EMAIL_KB_START_X;
  
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 10; c++) {
      const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      int bx = startX + c * keyW;
      int by = kbStartY + r * keyH;
      
      if (strcmp(label, "SPACE") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "SPACE") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "SPACE") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, 0x2104); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "SPACE", COL_GRAY, 1);
        } continue;
      }
      if (strcmp(label, "OK") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "OK") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "OK") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, getThemePrimary()); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "ENTER", COL_WHITE, 1);
        } continue;
      }
      uint16_t btnCol = getThemeSecondary();
      if (strcmp(label, "DEL") == 0 || strcmp(label, "SHIFT") == 0 || strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) btnCol = getThemePrimary();
      lcdFillRect(bx, by, keyW, keyH, btnCol); lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemeBg());
      char disp[8]; strncpy(disp, label, 7); disp[7] = 0;
      if (!shiftActive && disp[0] >= 'A' && disp[0] <= 'Z' && strlen(disp) == 1) disp[0] += 32;
      lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, disp, COL_WHITE, 1);
      if (strcmp(label, "SHIFT") == 0 && shiftActive) { lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemePrimary()); lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, "SHIFT", COL_WHITE, 1); }
    }
  }
  
  drawControlBar();
}

void handleEmailComposeSend() {
  if (strlen(composeTo) > 0 && strlen(composeSubject) > 0) {
    // Try to send real email via SMTP
    showNotification("Sending email...");
    bool emailSent = sendEmail(composeTo, composeSubject, composeBody);
    
    if (emailSent) {
      // Add to sent emails for local record
      if (emailCount < MAX_EMAILS) {
        EmailMessage sentEmail;
        strncpy(sentEmail.sender, gmailUsername, sizeof(sentEmail.sender));
        strncpy(sentEmail.subject, composeSubject, sizeof(sentEmail.subject));
        strncpy(sentEmail.body, composeBody, sizeof(sentEmail.body));
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 100)) {
          strftime(sentEmail.timestamp, sizeof(sentEmail.timestamp), "%H:%M", &timeinfo);
        } else {
          strcpy(sentEmail.timestamp, "--:--");
        }
        sentEmail.unread = false;
        sentEmail.isLocal = true;
        
        emails[emailCount] = sentEmail;
        emailCount++;
      }
      
      showNotification("Email sent successfully");
      composeTo[0] = 0;
      composeSubject[0] = 0;
      composeBody[0] = 0;
      composeField = 0;
      currentMode = MODE_EMAIL_INBOX;
      drawEmailInbox();
    }
    // Error notifications are handled inside sendEmail()
  } else {
    showNotification("Please fill To and Subject");
  }
}

void handleEmailComposeTouch(uint16_t tx, uint16_t ty) {
  // Check if touch is in control bar area first
  if (ty >= BAR_Y) {
    handleControlBarTouch(tx, ty);
    return;
  }
  
  // Back button (sits just above the control bar, clear of the keyboard)
  if (tx >= 10 && tx < 80 && ty >= EMAIL_COMPOSE_BTN_Y && ty < EMAIL_COMPOSE_BTN_Y + EMAIL_COMPOSE_BTN_H) {
    uiClick();
    currentMode = MODE_EMAIL_INBOX;
    drawEmailInbox();
    return;
  }
  
  // Send button (sits just above the control bar, clear of the keyboard)
  if (tx >= LCD_WIDTH - 80 && tx < LCD_WIDTH - 10 && ty >= EMAIL_COMPOSE_BTN_Y && ty < EMAIL_COMPOSE_BTN_Y + EMAIL_COMPOSE_BTN_H) {
    uiClick();
    handleEmailComposeSend();
    return;
  }
  
  // Field selection (must match the layout used in drawEmailCompose)
  int fieldY = 30;
  int subjY = fieldY + 35;
  int bodyY = subjY + 35;
  
  if (tx >= 40 && tx < LCD_WIDTH - 90 && ty >= fieldY - 2 && ty < fieldY + 22) {
    composeField = 0;
    drawEmailCompose();
    return;
  }
  
  if (tx >= 80 && tx < LCD_WIDTH - 90 && ty >= subjY - 2 && ty < subjY + 22) {
    composeField = 1;
    drawEmailCompose();
    return;
  }
  
  if (tx >= 10 && tx < LCD_WIDTH - 10 && ty >= bodyY + 20 && ty < bodyY + 50) {
    composeField = 2;
    drawEmailCompose();
    return;
  }
  
  // Keyboard handling (reuse existing keyboard system)
  int kbStartY = EMAIL_KB_START_Y;
  int keyW = EMAIL_KB_KEY_W;
  int keyH = EMAIL_KB_KEY_H;
  int startX = EMAIL_KB_START_X;
  
  if (ty >= kbStartY) {
    int r = (ty - kbStartY) / keyH;
    int c = (tx - startX) / keyW;
    if (r >= 0 && r < 4 && c >= 0 && c < 10 && tx >= startX) {
      playKeyBeep();
      const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      
      char* targetField = NULL;
      int targetFieldSize = 0;
      
      if (composeField == 0) { targetField = composeTo; targetFieldSize = sizeof(composeTo); }
      else if (composeField == 1) { targetField = composeSubject; targetFieldSize = sizeof(composeSubject); }
      else if (composeField == 2) { targetField = composeBody; targetFieldSize = sizeof(composeBody); }
      
      if (targetField) {
        int maxLen = targetFieldSize - 1; // leave room for null terminator
        if (strcmp(label, "DEL") == 0) {
          int len = strlen(targetField);
          if (len > 0) {
            targetField[len - 1] = 0;
          }
        } else if (strcmp(label, "SHIFT") == 0) {
          shiftActive = !shiftActive;
        } else if (strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) {
          isNumLayout = !isNumLayout;
        } else if (strcmp(label, "SPACE") == 0) {
          int len = strlen(targetField);
          if (len < maxLen) {
            targetField[len] = ' ';
            targetField[len + 1] = 0;
          }
        } else if (strcmp(label, "OK") == 0) {
          if (composeField < 2) {
            composeField++;
          } else {
            handleEmailComposeSend();
            return;
          }
        } else {
          char ch = label[0];
          if (!shiftActive && ch >= 'A' && ch <= 'Z') ch += 32;
          int len = strlen(targetField);
          if (len < maxLen) {
            targetField[len] = ch;
            targetField[len + 1] = 0;
          }
          if (shiftActive) shiftActive = false;
        }
        
        drawEmailCompose();
        return;
      }
    }
  }
}

// ---------- W Shop: GitHub repo .cpp file browser ----------
// Opening W Shop scans the repo's file tree for .cpp files over HTTPS and
// lists them. Per file you can DOWNLOAD it to the SD card, and once
// downloaded, UNINSTALL (delete it again) or CHECK it.
//
// IMPORTANT HONEST LIMITATION: "CHECK" is a lightweight heuristic scan
// (brace/paren/bracket balance + presence of at least one semicolon), not a
// real compiler pass. The ESP32-S3 has no C/C++ compiler or linker on the
// device, so nothing here actually compiles the file or performs true syntax
// checking (that would require a full toolchain, which doesn't run on this
// chip) - it's a best-effort sanity check only, and is labeled as such on
// screen so it's never mistaken for "this file will build."
#define COL_CYAN 0x07FF
const char* WSHOP_REPO_OWNER = "connorac2013-tech";
const char* WSHOP_REPO_NAME  = "wave-o.s";
#define WSHOP_MAX_CPP_FILES 80
#define WSHOP_PATH_MAXLEN 96
char wshopCppFiles[WSHOP_MAX_CPP_FILES][WSHOP_PATH_MAXLEN];
bool wshopFileDownloaded[WSHOP_MAX_CPP_FILES] = {false};
int  wshopCppFileCount = 0;
char wshopFetchStatus[80] = "";
bool wshopFetchOk = false;
int  wshopRepoScrollOffset = 0;
char wshopUsedBranch[8] = "main";

// Turns a repo path like "src/foo/bar.cpp" into a flat SD filename like
// "/wshop_src_foo_bar.cpp" so nested paths don't need real subfolders.
void wshopLocalPath(int idx, char* out, size_t outLen) {
  char tmp[WSHOP_PATH_MAXLEN];
  strncpy(tmp, wshopCppFiles[idx], sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = '\0';
  for (char* p = tmp; *p; p++) if (*p == '/') *p = '_';
  snprintf(out, outLen, "/wshop_%s", tmp);
}

// Reads the HTTPS JSON response body from the GitHub trees API one char at a
// time (no buffering the whole response - trees can be large) looking for
// `"path":"...."` entries whose value ends in ".cpp", and collects them.
bool wshopFetchCppFileList() {
  wshopCppFileCount = 0; wshopRepoScrollOffset = 0; wshopFetchOk = false;
  if (WiFi.status() != WL_CONNECTED) { strncpy(wshopFetchStatus, "NOT CONNECTED TO WIFI", sizeof(wshopFetchStatus)); return false; }

  const char* branches[] = {"main", "master"};
  for (int b = 0; b < 2; b++) {
    WiFiClientSecure client;
    client.setInsecure(); // GitHub's cert chain isn't pinned/embedded on-device
    HTTPClient http;
    char url[160];
    snprintf(url, sizeof(url), "https://api.github.com/repos/%s/%s/git/trees/%s?recursive=1", WSHOP_REPO_OWNER, WSHOP_REPO_NAME, branches[b]);
    if (!http.begin(client, url)) { strncpy(wshopFetchStatus, "COULD NOT OPEN CONNECTION", sizeof(wshopFetchStatus)); continue; }
    http.addHeader("User-Agent", "WaveOS-WShop");
    http.addHeader("Accept", "application/vnd.github+json");
    int code = http.GET();
    if (code != 200) {
      char msg[80]; snprintf(msg, sizeof(msg), "HTTP %d ON BRANCH %s", code, branches[b]);
      strncpy(wshopFetchStatus, msg, sizeof(wshopFetchStatus));
      http.end();
      continue; // try next branch (e.g. repo uses master, or is private/renamed/missing)
    }

    // Stream-scan for "path":"....." occurrences
    WiFiClient* stream = http.getStreamPtr();
    const char* key = "\"path\":\"";
    int keyPos = 0; bool inValue = false;
    char valBuf[WSHOP_PATH_MAXLEN]; int valLen = 0;
    unsigned long lastByte = millis();
    while (http.connected() && (stream->available() || millis() - lastByte < 5000)) {
      if (!stream->available()) { delay(2); continue; }
      char c = stream->read(); lastByte = millis();
      if (!inValue) {
        if (c == key[keyPos]) { keyPos++; if (key[keyPos] == '\0') { inValue = true; valLen = 0; keyPos = 0; } }
        else { keyPos = (c == key[0]) ? 1 : 0; }
      } else {
        if (c == '"') {
          valBuf[valLen] = '\0'; inValue = false;
          int vlen = valLen;
          if (vlen >= 4 && strcmp(valBuf + vlen - 4, ".cpp") == 0 && wshopCppFileCount < WSHOP_MAX_CPP_FILES) {
            strncpy(wshopCppFiles[wshopCppFileCount], valBuf, WSHOP_PATH_MAXLEN - 1);
            wshopCppFiles[wshopCppFileCount][WSHOP_PATH_MAXLEN - 1] = '\0';
            wshopCppFileCount++;
          }
        } else if (valLen < WSHOP_PATH_MAXLEN - 1) {
          valBuf[valLen++] = c;
        } else { inValue = false; } // path longer than buffer - drop it, keep scanning
        if (wshopCppFileCount >= WSHOP_MAX_CPP_FILES) break;
      }
    }
    http.end();

    if (wshopCppFileCount > 0) {
      strncpy(wshopUsedBranch, branches[b], sizeof(wshopUsedBranch) - 1); wshopUsedBranch[sizeof(wshopUsedBranch) - 1] = '\0';
      char msg[80]; snprintf(msg, sizeof(msg), "FOUND %d .cpp FILE(S) ON %s", wshopCppFileCount, branches[b]);
      strncpy(wshopFetchStatus, msg, sizeof(wshopFetchStatus));
      wshopFetchOk = true;
      // Mark which ones are already sitting on the SD card from a prior download
      for (int i = 0; i < wshopCppFileCount; i++) {
        if (!sdCardMounted) { wshopFileDownloaded[i] = false; continue; }
        char localPath[80]; wshopLocalPath(i, localPath, sizeof(localPath));
        wshopFileDownloaded[i] = SD_MMC.exists(localPath);
      }
      return true;
    }
    strncpy(wshopFetchStatus, "NO .cpp FILES FOUND", sizeof(wshopFetchStatus));
    wshopFetchOk = true; // request succeeded, repo just has none / branch empty
  }
  return wshopFetchOk;
}

// Cyan globe glyph with a thick red diagonal slash through it - the
// no-internet indicator. Built entirely from existing primitives (circles +
// filled rects for the meridian/equator lines + a few parallel diagonal
// lines for the slash, so it stays crisp at small sizes).
void drawNoInternetIcon(int cx, int cy, int r) {
  lcdDrawCircle(cx, cy, r, COL_CYAN);
  lcdDrawCircle(cx, cy, r - 1, COL_CYAN);
  lcdFillRect(cx - r, cy - 1, r * 2, 2, COL_CYAN);   // equator
  lcdFillRect(cx - 1, cy - r, 2, r * 2, COL_CYAN);   // prime meridian
  lcdDrawCircle(cx, cy, (r * 2) / 3, COL_CYAN);      // inner longitude ring
  for (int t = -2; t <= 2; t++) {
    lcdDrawLine(cx - r + 3 + t, cy - r + 3, cx + r - 3 + t, cy + r - 3, COL_RED);
  }
}

// Same globe glyph without the red slash (we're online whenever this is
// shown), used as a plain "working on it" screen while a blocking network
// call (search / download) is in flight.
void drawWShopLoading(const char* msg) {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary());
  lcdDrawTextCentered(0, LCD_WIDTH, 10, "W SHOP", COL_WHITE, 1);
  int cx = LCD_WIDTH / 2, cy = 140, r = 44;
  lcdDrawCircle(cx, cy, r, COL_CYAN); lcdDrawCircle(cx, cy, r - 1, COL_CYAN);
  lcdFillRect(cx - r, cy - 1, r * 2, 2, COL_CYAN);
  lcdFillRect(cx - 1, cy - r, 2, r * 2, COL_CYAN);
  lcdDrawCircle(cx, cy, (r * 2) / 3, COL_CYAN);
  lcdDrawTextCentered(0, LCD_WIDTH, cy + r + 20, msg, COL_WHITE, 1);
  char repoLine[64]; snprintf(repoLine, sizeof(repoLine), "%s/%s", WSHOP_REPO_OWNER, WSHOP_REPO_NAME);
  lcdDrawTextCentered(0, LCD_WIDTH, cy + r + 36, repoLine, COL_GRAY, 1);
}

// Streams a file straight from raw.githubusercontent.com to the SD card
// while tallying brace/paren/bracket balance and semicolon presence as it
// goes, for the heuristic check below.
bool wshopDownloadFile(int idx) {
  if (idx < 0 || idx >= wshopCppFileCount) return false;
  if (!sdCardMounted) { strncpy(wshopFetchStatus, "NO SD CARD MOUNTED", sizeof(wshopFetchStatus)); return false; }
  if (WiFi.status() != WL_CONNECTED) { strncpy(wshopFetchStatus, "NOT CONNECTED TO WIFI", sizeof(wshopFetchStatus)); return false; }

  char url[220];
  snprintf(url, sizeof(url), "https://raw.githubusercontent.com/%s/%s/%s/%s", WSHOP_REPO_OWNER, WSHOP_REPO_NAME, wshopUsedBranch, wshopCppFiles[idx]);
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url)) { strncpy(wshopFetchStatus, "COULD NOT OPEN CONNECTION", sizeof(wshopFetchStatus)); return false; }
  http.addHeader("User-Agent", "WaveOS-WShop");
  int code = http.GET();
  if (code != 200) {
    char m[80]; snprintf(m, sizeof(m), "DOWNLOAD FAILED (HTTP %d)", code);
    strncpy(wshopFetchStatus, m, sizeof(wshopFetchStatus)); http.end(); return false;
  }
  char localPath[80]; wshopLocalPath(idx, localPath, sizeof(localPath));
  File f = SD_MMC.open(localPath, "w");
  if (!f) { strncpy(wshopFetchStatus, "SD WRITE FAILED", sizeof(wshopFetchStatus)); http.end(); return false; }

  WiFiClient* stream = http.getStreamPtr();
  int braceBal = 0, parenBal = 0, bracketBal = 0; bool sawSemicolon = false; size_t bytes = 0;
  unsigned long lastByte = millis();
  uint8_t buf[256];
  while (http.connected() && (stream->available() || millis() - lastByte < 5000)) {
    if (!stream->available()) { delay(2); continue; }
    size_t avail = stream->available();
    int n = stream->readBytes(buf, avail > sizeof(buf) ? sizeof(buf) : avail);
    if (n <= 0) continue;
    lastByte = millis();
    f.write(buf, n); bytes += n;
    for (int i = 0; i < n; i++) {
      char c = (char)buf[i];
      if (c == '{') braceBal++; else if (c == '}') braceBal--;
      else if (c == '(') parenBal++; else if (c == ')') parenBal--;
      else if (c == '[') bracketBal++; else if (c == ']') bracketBal--;
      else if (c == ';') sawSemicolon = true;
    }
  }
  f.close(); http.end();
  wshopFileDownloaded[idx] = true;
  bool balanced = (braceBal == 0 && parenBal == 0 && bracketBal == 0);
  char m[80];
  snprintf(m, sizeof(m), "DOWNLOADED %uB - BASIC CHECK: %s", (unsigned)bytes, balanced ? (sawSemicolon ? "OK" : "OK (NO ';')") : "MISMATCHED { } / ( ) / [ ]");
  strncpy(wshopFetchStatus, m, sizeof(wshopFetchStatus));
  return true;
}

// Heuristic re-check of an already-downloaded file. NOT a compiler - just
// brace/paren/bracket balance and semicolon presence, so it can catch a
// truncated download or obviously broken file, nothing more.
bool wshopCheckFileSyntax(int idx) {
  if (idx < 0 || idx >= wshopCppFileCount) return false;
  char localPath[80]; wshopLocalPath(idx, localPath, sizeof(localPath));
  File f = SD_MMC.open(localPath, "r");
  if (!f) { strncpy(wshopFetchStatus, "FILE NOT FOUND ON SD", sizeof(wshopFetchStatus)); return false; }
  int braceBal = 0, parenBal = 0, bracketBal = 0; bool sawSemicolon = false; size_t bytes = 0;
  while (f.available()) {
    char c = f.read(); bytes++;
    if (c == '{') braceBal++; else if (c == '}') braceBal--;
    else if (c == '(') parenBal++; else if (c == ')') parenBal--;
    else if (c == '[') bracketBal++; else if (c == ']') bracketBal--;
    else if (c == ';') sawSemicolon = true;
  }
  f.close();
  bool balanced = (braceBal == 0 && parenBal == 0 && bracketBal == 0);
  char m[80];
  snprintf(m, sizeof(m), "BASIC CHECK (%uB): %s", (unsigned)bytes, balanced ? (sawSemicolon ? "OK" : "OK (NO ';')") : "MISMATCHED { } / ( ) / [ ]");
  strncpy(wshopFetchStatus, m, sizeof(wshopFetchStatus));
  return balanced;
}

void wshopUninstallFile(int idx) {
  if (idx < 0 || idx >= wshopCppFileCount) return;
  char localPath[80]; wshopLocalPath(idx, localPath, sizeof(localPath));
  SD_MMC.remove(localPath);
  wshopFileDownloaded[idx] = false;
  strncpy(wshopFetchStatus, "UNINSTALLED - FILE REMOVED FROM SD", sizeof(wshopFetchStatus));
}

#define WSHOP_REPO_ROW_H 46
#define WSHOP_REPO_LIST_Y0 76
// Opens W Shop: if online, shows the loading screen and kicks off the repo
// scan automatically; if offline, the no-internet icon is drawn instead and
// no network call is attempted.
void openWShop() {
  currentMode = MODE_WSHOP;
  if (WiFi.status() == WL_CONNECTED) {
    drawWShopLoading("SEARCHING REPOSITORY...");
    wshopFetchCppFileList();
  }
  drawWShop();
}

void drawWShop() {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary());
  lcdDrawText(10, 10, "< BACK", COL_WHITE, 1);
  lcdDrawTextCentered(50, LCD_WIDTH - 130, 10, "W SHOP", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH - 74, 4, 66, 26, getThemeSecondary());
  lcdDrawTextCentered(LCD_WIDTH - 74, 66, 12, "REFRESH", COL_WHITE, 1);

  if (WiFi.status() != WL_CONNECTED) {
    drawNoInternetIcon(LCD_WIDTH / 2, 150, 46);
    lcdDrawTextCentered(0, LCD_WIDTH, 210, "NO INTERNET CONNECTION", COL_YELLOW, 1);
    lcdDrawTextCentered(0, LCD_WIDTH, 226, "CONNECT TO WIFI TO BROWSE THE REPO", COL_GRAY, 1);
    drawControlBar();
    return;
  }

  char repoLine[64]; snprintf(repoLine, sizeof(repoLine), "%s/%s", WSHOP_REPO_OWNER, WSHOP_REPO_NAME);
  lcdDrawText(10, 42, repoLine, COL_GRAY, 1);
  lcdDrawText(10, 58, wshopFetchStatus, wshopFetchOk ? COL_GREEN : COL_YELLOW, 1);

  int listY = WSHOP_REPO_LIST_Y0;
  int visibleRows = (BAR_Y - listY) / WSHOP_REPO_ROW_H;
  for (int i = 0; i < visibleRows; i++) {
    int idx = i + wshopRepoScrollOffset;
    if (idx >= wshopCppFileCount) break;
    int rowY = listY + i * WSHOP_REPO_ROW_H;
    lcdFillRect(10, rowY, LCD_WIDTH - 20, WSHOP_REPO_ROW_H - 4, getThemeSecondary());
    lcdDrawText(16, rowY + 5, wshopCppFiles[idx], COL_WHITE, 1);
    int btnY = rowY + 20, btnH = 20;
    if (wshopFileDownloaded[idx]) {
      int checkW = 84, checkX = LCD_WIDTH - 190;
      lcdFillRect(checkX, btnY, checkW, btnH, getThemePrimary());
      lcdDrawTextCentered(checkX, checkW, btnY + 5, "CHECK", COL_WHITE, 1);
      int unW = 90, unX = LCD_WIDTH - 100;
      lcdFillRect(unX, btnY, unW, btnH, COL_RED); lcdFillRect(unX + 2, btnY + 2, unW - 4, btnH - 4, getThemeBg());
      lcdDrawTextCentered(unX, unW, btnY + 5, "UNINSTALL", COL_RED, 1);
    } else {
      int dlW = 90, dlX = LCD_WIDTH - 100;
      lcdFillRect(dlX, btnY, dlW, btnH, COL_GREEN); lcdFillRect(dlX + 2, btnY + 2, dlW - 4, btnH - 4, getThemeBg());
      lcdDrawTextCentered(dlX, dlW, btnY + 5, "DOWNLOAD", COL_GREEN, 1);
    }
  }
  if (wshopCppFileCount > visibleRows) lcdDrawText(LCD_WIDTH - 60, BAR_Y - 16, "SCROLL", COL_GRAY, 1);
  drawControlBar();
}

void handleWShopTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  if (ty < 36) { uiClick(); currentMode = MODE_LAUNCHER; drawLauncher(); return; }
  if (tx >= LCD_WIDTH - 74 && tx < LCD_WIDTH && ty >= 4 && ty < 30) {
    uiClick();
    if (WiFi.status() == WL_CONNECTED) {
      drawWShopLoading("SEARCHING REPOSITORY...");
      wshopFetchCppFileList();
    }
    drawWShop();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) return; // nothing else is interactive offline

  int listY = WSHOP_REPO_LIST_Y0;
  int visibleRows = (BAR_Y - listY) / WSHOP_REPO_ROW_H;
  for (int i = 0; i < visibleRows; i++) {
    int idx = i + wshopRepoScrollOffset;
    if (idx >= wshopCppFileCount) break;
    int rowY = listY + i * WSHOP_REPO_ROW_H;
    int btnY = rowY + 20, btnH = 20;
    if (ty < btnY || ty >= btnY + btnH) continue;
    if (wshopFileDownloaded[idx]) {
      int checkW = 84, checkX = LCD_WIDTH - 190;
      int unW = 90, unX = LCD_WIDTH - 100;
      if (tx >= checkX && tx < checkX + checkW) { uiClick(); wshopCheckFileSyntax(idx); drawWShop(); return; }
      if (tx >= unX && tx < unX + unW) { uiClick(); wshopUninstallFile(idx); playRemoveApp(); drawWShop(); return; }
    } else {
      int dlW = 90, dlX = LCD_WIDTH - 100;
      if (tx >= dlX && tx < dlX + dlW) {
        uiClick();
        drawWShopLoading("DOWNLOADING FILE...");
        if (wshopDownloadFile(idx)) playSuccess();
        drawWShop();
        return;
      }
    }
  }
  // Tap anywhere else in the list area scrolls, if there's more than one page
  if (wshopCppFileCount > visibleRows && ty >= listY) {
    uiClick();
    wshopRepoScrollOffset += visibleRows;
    if (wshopRepoScrollOffset >= wshopCppFileCount) wshopRepoScrollOffset = 0;
    drawWShop();
  }
}
// ---------- Gmail Sign-in Screen ----------
char gmailUsernameEntry[64] = "";
char gmailPasswordEntry[64] = "";
bool gmailEnteringPassword = false;
bool gmailShiftActive = false;
bool gmailIsNumLayout = false;

void drawGmailSignin() {
  lcdFillScreen(getThemeBg());
  lcdDrawTextCentered(0, LCD_WIDTH, 20, "GMAIL SETUP", COL_WHITE, 1);
  
  int fieldY = 40;
  // Username field
  lcdFillRect(30, fieldY, LCD_WIDTH-60, 30, getThemeSecondary());
  lcdDrawText(40, fieldY + 8, "USERNAME:", COL_WHITE, 1);
  lcdFillRect(140, fieldY + 5, LCD_WIDTH-180, 20, COL_BLACK);
  lcdDrawText(145, fieldY + 8, gmailUsernameEntry, COL_WHITE, 1);
  
  // Password field
  int passY = fieldY + 40;
  lcdFillRect(30, passY, LCD_WIDTH-60, 30, getThemeSecondary());
  lcdDrawText(40, passY + 8, "PASSWORD:", COL_WHITE, 1);
  lcdFillRect(140, passY + 5, LCD_WIDTH-180, 20, COL_BLACK);
  
  if (gmailEnteringPassword) {
    // Show dots for each character in password
    int passLen = strlen(gmailPasswordEntry);
    for (int i = 0; i < passLen; i++) {
      int dotX = 145 + i * 8;
      if (dotX < LCD_WIDTH - 50) {
        lcdFillCircle(dotX, passY + 16, 3, COL_WHITE);
      }
    }
  }
  
  // Field selection indicator
  if (!gmailEnteringPassword) {
    lcdDrawRect(139, fieldY + 4, LCD_WIDTH-178, 22, COL_YELLOW);
  } else {
    lcdDrawRect(139, passY + 4, LCD_WIDTH-178, 22, COL_YELLOW);
  }
  
  // Use existing keyboard system
  int kbStartY = 115;
  int keyW = 42;
  int keyH = 42;
  int startX = 30;
  
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 10; c++) {
      const char* label = gmailIsNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      int bx = startX + c * keyW;
      int by = kbStartY + r * keyH;
      
      if (strcmp(label, "SPACE") == 0) {
        if (c == 0 || strcmp(gmailIsNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "SPACE") != 0) {
          int span = 1; while (c + span < 10 && strcmp(gmailIsNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "SPACE") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, 0x2104); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "SPACE", COL_GRAY, 1);
        } continue;
      }
      if (strcmp(label, "OK") == 0) {
        if (c == 0 || strcmp(gmailIsNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "OK") != 0) {
          int span = 1; while (c + span < 10 && strcmp(gmailIsNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "OK") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, COL_GREEN); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "OK", COL_WHITE, 1);
        } continue;
      }
      uint16_t btnCol = getThemeSecondary();
      if (strcmp(label, "DEL") == 0 || strcmp(label, "SHIFT") == 0 || strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) btnCol = getThemePrimary();
      lcdFillRect(bx, by, keyW, keyH, btnCol); lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemeBg());
      char disp[8]; strncpy(disp, label, 7); disp[7] = 0;
      if (!gmailShiftActive && disp[0] >= 'A' && disp[0] <= 'Z' && strlen(disp) == 1) disp[0] += 32;
      lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, disp, COL_WHITE, 1);
      if (strcmp(label, "SHIFT") == 0 && gmailShiftActive) { lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemePrimary()); lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, "SHIFT", COL_WHITE, 1); }
    }
  }
  
  drawControlBar();
}

bool handleGmailSigninTouch(uint16_t tx, uint16_t ty) {
  int fieldY = 40;
  int passY = fieldY + 40;
  
  // Username field touch
  if (tx >= 140 && tx < LCD_WIDTH-40 && ty >= fieldY + 5 && ty < fieldY + 25) {
    gmailEnteringPassword = false;
    drawGmailSignin();
    return true;
  }
  
  // Password field touch
  if (tx >= 140 && tx < LCD_WIDTH-40 && ty >= passY + 5 && ty < passY + 25) {
    gmailEnteringPassword = true;
    drawGmailSignin();
    return true;
  }
  
  // Keyboard touch using existing system
  int kbStartY = 115;
  int keyW = 42;
  int keyH = 42;
  int startX = 30;
  
  if (ty >= kbStartY) {
    int r = (ty - kbStartY) / keyH;
    int c = (tx - startX) / keyW;
    if (r >= 0 && r < 4 && c >= 0 && c < 10 && tx >= startX) {
      playKeyBeep();
      const char* label = gmailIsNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      
      char* targetEntry = gmailEnteringPassword ? gmailPasswordEntry : gmailUsernameEntry;
      int currentLen = strlen(targetEntry);
      
      if (strcmp(label, "DEL") == 0) {
        if (currentLen > 0) {
          targetEntry[currentLen - 1] = 0;
        }
      } else if (strcmp(label, "SHIFT") == 0) {
        gmailShiftActive = !gmailShiftActive;
      } else if (strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) {
        gmailIsNumLayout = !gmailIsNumLayout;
      } else if (strcmp(label, "SPACE") == 0) {
        if (currentLen < 63) {
          targetEntry[currentLen] = ' ';
          targetEntry[currentLen + 1] = 0;
        }
      } else if (strcmp(label, "OK") == 0) {
        if (strlen(gmailUsernameEntry) > 0 && strlen(gmailPasswordEntry) > 0) {
          // Strip spaces from the password: Google displays App Passwords with
          // spaces purely for readability ("abcd efgh ijkl mnop") - the actual
          // credential has no spaces. Saving them verbatim silently breaks
          // SMTP AUTH LOGIN every time ("AUTH FAILED: CHECK PASSWORD").
          char cleanPass[64]; int cpi = 0;
          for (int si = 0; gmailPasswordEntry[si] != 0 && cpi < 63; si++) {
            if (gmailPasswordEntry[si] != ' ') cleanPass[cpi++] = gmailPasswordEntry[si];
          }
          cleanPass[cpi] = 0;
          strncpy(gmailUsername, gmailUsernameEntry, sizeof(gmailUsername)-1);
          strncpy(gmailPassword, cleanPass, sizeof(gmailPassword)-1);
          gmailConfigured = true;
          firstBootCompleted = true;
          saveSettings();
          currentMode = MODE_LAUNCHER;
          drawLauncher();
          showNotification("Gmail configured!");
        } else {
          showNotification("Please enter both fields");
        }
      } else {
        char ch = label[0];
        if (!gmailShiftActive && ch >= 'A' && ch <= 'Z') ch += 32;
        if (currentLen < 63) {
          targetEntry[currentLen] = ch;
          targetEntry[currentLen + 1] = 0;
        }
        if (gmailShiftActive) gmailShiftActive = false;
      }
      
      drawGmailSignin();
      return true;
    }
  }
  
  return false;
}
void drawClock() {
  lcdFillScreen(getThemeBg());
  lcdDrawTextCentered(0, LCD_WIDTH, 40, "CLOCK", COL_WHITE, 1);
  strcpy(clkLastStr, ""); clkSynced = false; ntpSyncNow(); drawControlBar();
}
void updateClock() {
  struct tm timeinfo; char buf[24];
  if (getLocalTime(&timeinfo, 200)) { clkSynced = true; strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo); }
  else if (WiFi.status() == WL_CONNECTED) { strcpy(buf, "SYNCING..."); } else { strcpy(buf, "NO WIFI"); }
  if (strcmp(buf, clkLastStr) != 0) {
    lcdFillRect(0, 120, LCD_WIDTH, 40, getThemeBg());
    lcdDrawTextCentered(0, LCD_WIDTH, 130, buf, COL_YELLOW, clkSynced ? 3 : 2);
    strcpy(clkLastStr, buf);
    if (clkSynced) { char dateBuf[32]; strftime(dateBuf, sizeof(dateBuf), "%A, %B %d %Y", &timeinfo);
      lcdFillRect(0, 175, LCD_WIDTH, 16, getThemeBg()); lcdDrawTextCentered(0, LCD_WIDTH, 178, dateBuf, COL_WHITE, 1); }
  }
}
void handleClockTouch(uint16_t tx, uint16_t ty) {}

// ---------- Tic Tac Toe ----------
const int TTT_ROWS = 3, TTT_COLS = 3; uint8_t board[TTT_ROWS][TTT_COLS]; uint8_t currentPlayerTTT = 1; bool tttGameOver = false;
const int TTT_CELL_W = 300 / TTT_COLS; const int TTT_CELL_H = 240 / TTT_ROWS; const int TTT_OFFSET_X = 90; const int TTT_OFFSET_Y = 20;
void drawGrid() { for (int i = 1; i < TTT_COLS; i++) lcdFillRect(TTT_OFFSET_X + i*TTT_CELL_W - 2, TTT_OFFSET_Y, 4, 240, 0xFFFF); for (int i = 1; i < TTT_ROWS; i++) lcdFillRect(TTT_OFFSET_X, TTT_OFFSET_Y + i*TTT_CELL_H - 2, 300, 4, 0xFFFF); }
void drawBoard() {
  lcdFillScreen(getThemeBg()); drawGrid();
  for (int r = 0; r < TTT_ROWS; r++) { for (int c = 0; c < TTT_COLS; c++) { int x = TTT_OFFSET_X + c*TTT_CELL_W, y = TTT_OFFSET_Y + r*TTT_CELL_H; int cx = x + TTT_CELL_W/2, cy = y + TTT_CELL_H/2;
      if (board[r][c] == 1) { lcdDrawLine(x+10, y+10, x+TTT_CELL_W-10, y+TTT_CELL_H-10, 0xF800); lcdDrawLine(x+TTT_CELL_W-10, y+10, x+10, y+TTT_CELL_H-10, 0xF800); }
      else if (board[r][c] == 2) { lcdDrawCircle(cx, cy, TTT_CELL_W/2 - 15, 0x07E0); } } }
  drawControlBar(); if (tttGameOver) { uint8_t w = checkWinner(); const char* msg = w == 1 ? "X WINS!" : w == 2 ? "O WINS!" : "DRAW!"; lcdDrawTextCentered(TTT_OFFSET_X, 300, TTT_OFFSET_Y + 240 + 4, msg, COL_WHITE, 1); }
}
uint8_t checkWinner() { for (int r = 0; r < 3; r++) if (board[r][0] && board[r][0]==board[r][1] && board[r][1]==board[r][2]) return board[r][0]; for (int c = 0; c < 3; c++) if (board[0][c] && board[0][c]==board[1][c] && board[1][c]==board[2][c]) return board[0][c]; if (board[0][0] && board[0][0]==board[1][1] && board[1][1]==board[2][2]) return board[0][0]; if (board[0][2] && board[0][2]==board[1][1] && board[1][1]==board[2][0]) return board[0][2]; return 0; }
bool boardFull() { for (int r=0;r<3;r++) for (int c=0;c<3;c++) if (!board[r][c]) return false; return true; }
void resetGame() { memset(board, 0, sizeof(board)); currentPlayerTTT = 1; tttGameOver = false; drawBoard(); }
int minimax(int depth, bool isMax) { uint8_t w = checkWinner(); if (w == 2) return 10-depth; if (w == 1) return depth-10; if (boardFull()) return 0;
  if (isMax) { int best = -1000; for (int r=0;r<3;r++) for (int c=0;c<3;c++) if (!board[r][c]) { board[r][c]=2; best=max(best,minimax(depth+1,false)); board[r][c]=0; } return best; }
  else { int best = 1000; for (int r=0;r<3;r++) for (int c=0;c<3;c++) if (!board[r][c]) { board[r][c]=1; best=min(best,minimax(depth+1,true)); board[r][c]=0; } return best; } }
void makeAIMove() { int bestScore = -1000, bestR = -1, bestC = -1; for (int r=0;r<3;r++) for (int c=0;c<3;c++) if (!board[r][c]) { board[r][c]=2; int s = minimax(0,false); board[r][c]=0; if (s > bestScore) { bestScore=s; bestR=r; bestC=c; } } if (bestR != -1) { board[bestR][bestC] = 2; drawBoard(); } }
void handleTicTacToeTouch(uint16_t tx, uint16_t ty) {
  if (tttGameOver) { if (tx >= TTT_OFFSET_X && tx < TTT_OFFSET_X+300 && ty >= TTT_OFFSET_Y && ty < TTT_OFFSET_Y+240) resetGame(); return; } if (ty >= BAR_Y) return; if (tx < TTT_OFFSET_X || tx >= TTT_OFFSET_X+300 || ty < TTT_OFFSET_Y || ty >= TTT_OFFSET_Y+240) return;
  int col = (tx - TTT_OFFSET_X) / TTT_CELL_W, row = (ty - TTT_OFFSET_Y) / TTT_CELL_H;
  if (board[row][col] == 0) { board[row][col] = currentPlayerTTT; drawBoard(); uint8_t w1 = checkWinner(); if (w1 || boardFull()) { tttGameOver = true; if (w1) playWinChime(); drawBoard(); return; } currentPlayerTTT = (currentPlayerTTT == 1 ? 2 : 1);
    if (currentMode == MODE_TICTACTOE_AI && currentPlayerTTT == 2) { delay(400); makeAIMove(); uint8_t w2 = checkWinner(); if (w2 || boardFull()) { tttGameOver = true; if (w2) playWinChime(); drawBoard(); return; } currentPlayerTTT = 1; } } }

// ---------- Checkers (8x8) ----------
#define CK_CELL 30
#define CK_OFF_X 120
#define CK_OFF_Y 20
int8_t ckBoard[8][8]; int ckSelR = -1, ckSelC = -1; uint8_t ckTurn = 1; bool ckGameOver = false;
bool ckIsPlayer(int8_t p) { return p == 1 || p == 2; } bool ckIsAI(int8_t p) { return p == 3 || p == 4; }
bool ckInBounds(int r, int c) { return r >= 0 && r < 8 && c >= 0 && c < 8; }
void initCheckers() {
  memset(ckBoard, 0, sizeof(ckBoard));
  for (int r = 0; r < 3; r++) for (int c = 0; c < 8; c++) if ((r + c) % 2 == 1) ckBoard[r][c] = 3;
  for (int r = 5; r < 8; r++) for (int c = 0; c < 8; c++) if ((r + c) % 2 == 1) ckBoard[r][c] = 1;
  ckSelR = -1; ckSelC = -1; ckTurn = 1; ckGameOver = false;
}
int ckCountPieces(int side) { int n = 0; for (int r = 0; r < 8; r++) for (int c = 0; c < 8; c++) { int8_t p = ckBoard[r][c]; if (side == 1 && ckIsPlayer(p)) n++; if (side == 2 && ckIsAI(p)) n++; } return n; }
int ckValidateMove(int fr, int fc, int tr, int tc, int side, int *midR, int *midC) {
  if (!ckInBounds(fr, fc) || !ckInBounds(tr, tc)) return 0; int8_t p = ckBoard[fr][fc]; if (p == 0) return 0;
  bool mine = (side == 1) ? ckIsPlayer(p) : ckIsAI(p); if (!mine) return 0; if (ckBoard[tr][tc] != 0) return 0;
  int dr = tr - fr, dc = tc - fc; bool king = (p == 2 || p == 4); int dir = (side == 1) ? -1 : 1;
  if (abs(dr) == 1 && abs(dc) == 1) { if (!king && dr != dir) return 0; return 1; }
  if (abs(dr) == 2 && abs(dc) == 2) { int mr = fr + dr / 2, mc = fc + dc / 2; int8_t mid = ckBoard[mr][mc]; if (mid == 0) return 0;
    bool enemyMid = (side == 1) ? ckIsAI(mid) : ckIsPlayer(mid); if (!enemyMid) return 0; if (!king && dr != 2 * dir) return 0; *midR = mr; *midC = mc; return 2; } return 0;
}
void ckApplyMove(int fr, int fc, int tr, int tc, int type, int midR, int midC, int side) {
  int8_t p = ckBoard[fr][fc]; ckBoard[tr][tc] = p; ckBoard[fr][fc] = 0; if (type == 2) ckBoard[midR][midC] = 0;
  if (side == 1 && tr == 0 && p == 1) ckBoard[tr][tc] = 2; if (side == 2 && tr == 7 && p == 3) ckBoard[tr][tc] = 4;
}
void drawCheckersBoard() {
  lcdFillScreen(getThemeBg());
  for (int r = 0; r < 8; r++) for (int c = 0; c < 8; c++) {
      int x = CK_OFF_X + c * CK_CELL, y = CK_OFF_Y + r * CK_CELL; uint16_t base = ((r + c) % 2 == 0) ? 0xC618 : 0x4208;
      lcdFillRect(x, y, CK_CELL, CK_CELL, base);
      if (ckSelR == r && ckSelC == c) { lcdFillRect(x, y, CK_CELL, 2, COL_YELLOW); lcdFillRect(x, y, 2, CK_CELL, COL_YELLOW); lcdFillRect(x, y+CK_CELL-2, CK_CELL, 2, COL_YELLOW); lcdFillRect(x+CK_CELL-2, y, 2, CK_CELL, COL_YELLOW); }
      int8_t p = ckBoard[r][c]; if (p) { int cx = x + CK_CELL / 2, cy = y + CK_CELL / 2; uint16_t col = (p == 1 || p == 2) ? COL_RED : COL_BLACK;
        lcdFillCircle(cx, cy, CK_CELL / 2 - 4, col); if (p == 2 || p == 4) lcdDrawCircle(cx, cy, CK_CELL / 2 - 8, COL_YELLOW); }
    }
  const char* msg; if (ckGameOver) msg = (ckTurn == 1) ? "BLACK WINS! TAP TO RESTART" : "RED WINS! TAP TO RESTART";
  else msg = (ckTurn == 1) ? "RED'S TURN" : "BLACK'S TURN";
  lcdDrawTextCentered(CK_OFF_X, CK_CELL * 8, CK_OFF_Y + CK_CELL * 8 + 4, msg, COL_WHITE, 1); drawControlBar();
}
void ckAIMove() {
  int bfr = -1, bfc = -1, btr = -1, btc = -1, btype = 0, bmidR = 0, bmidC = 0; bool haveCapture = false;
  for (int fr = 0; fr < 8; fr++) for (int fc = 0; fc < 8; fc++) { int8_t p = ckBoard[fr][fc]; if (!ckIsAI(p)) continue;
      for (int dr = -2; dr <= 2; dr++) { if (dr == 0) continue; for (int dc = -2; dc <= 2; dc++) { if (dc == 0 || abs(dr) != abs(dc)) continue;
          int tr = fr + dr, tc = fc + dc, midR, midC; int type = ckValidateMove(fr, fc, tr, tc, 2, &midR, &midC);
          if (type == 2 && !haveCapture) { haveCapture = true; bfr = fr; bfc = fc; btr = tr; btc = tc; btype = 2; bmidR = midR; bmidC = midC; }
          else if (type == 1 && !haveCapture && btype == 0) { bfr = fr; bfc = fc; btr = tr; btc = tc; btype = 1; } } } }
  if (btype) { if (btype == 2) playJumpChime(); ckApplyMove(bfr, bfc, btr, btc, btype, bmidR, bmidC, 2); }
}
void handleCheckersTouch(uint16_t tx, uint16_t ty) {
  if (ckGameOver) { if (ty < BAR_Y) { uiClick(); initCheckers(); drawCheckersBoard(); } else handleControlBarTouch(tx, ty); return; }
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  int c = (tx - CK_OFF_X) / CK_CELL, r = (ty - CK_OFF_Y) / CK_CELL;
  if (r < 0 || r >= 8 || c < 0 || c >= 8) return; int8_t p = ckBoard[r][c]; bool mine = (ckTurn == 1) ? ckIsPlayer(p) : ckIsAI(p);
  if (ckSelR == -1) { if (mine) { ckSelR = r; ckSelC = c; uiClick(); drawCheckersBoard(); } return; }
  if (r == ckSelR && c == ckSelC) { ckSelR = -1; ckSelC = -1; drawCheckersBoard(); return; }
  int midR, midC; int type = ckValidateMove(ckSelR, ckSelC, r, c, ckTurn, &midR, &midC);
  if (type) { if (type == 2) playJumpChime(); else uiClick(); ckApplyMove(ckSelR, ckSelC, r, c, type, midR, midC, ckTurn);
    ckSelR = -1; ckSelC = -1;
    int otherSide = (ckTurn == 1) ? 2 : 1; if (ckCountPieces(otherSide) == 0) { ckGameOver = true; playSuccess(); drawCheckersBoard(); return; }
    ckTurn = otherSide; drawCheckersBoard();
    if (currentMode == MODE_CHECKERS_AI && ckTurn == 2) { delay(300); ckAIMove(); if (ckCountPieces(1) == 0) { ckGameOver = true; playError(); drawCheckersBoard(); return; } ckTurn = 1; drawCheckersBoard(); }
  } else if (mine) { ckSelR = r; ckSelC = c; drawCheckersBoard(); }
}

// ---------- Minesweeper (8x8) ----------
#define MS_SIZE 8
#define MS_CELL 30
#define MS_OFF_X 120
#define MS_OFF_Y 34
#define MS_MINES 10
bool msMine[MS_SIZE][MS_SIZE]; uint8_t msAdj[MS_SIZE][MS_SIZE]; bool msRevealed[MS_SIZE][MS_SIZE]; bool msFlagged[MS_SIZE][MS_SIZE];
bool msGameOver = false; bool msWon = false; bool msFlagMode = false;
void initMinesweeper() {
  memset(msMine, 0, sizeof(msMine)); memset(msAdj, 0, sizeof(msAdj)); memset(msRevealed, 0, sizeof(msRevealed)); memset(msFlagged, 0, sizeof(msFlagged));
  msGameOver = false; msWon = false; msFlagMode = false; randomSeed(micros());
  int placed = 0; while (placed < MS_MINES) { int r = random(MS_SIZE), c = random(MS_SIZE); if (!msMine[r][c]) { msMine[r][c] = true; placed++; } }
  for (int r = 0; r < MS_SIZE; r++) for (int c = 0; c < MS_SIZE; c++) { if (msMine[r][c]) continue; int n = 0;
      for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) { if (dr == 0 && dc == 0) continue; int rr = r + dr, cc = c + dc; if (rr >= 0 && rr < MS_SIZE && cc >= 0 && cc < MS_SIZE && msMine[rr][cc]) n++; } msAdj[r][c] = n; }
}
void msReveal(int r, int c) {
  if (r < 0 || r >= MS_SIZE || c < 0 || c >= MS_SIZE) return; if (msRevealed[r][c] || msFlagged[r][c]) return; msRevealed[r][c] = true;
  if (msAdj[r][c] == 0 && !msMine[r][c]) for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) { if (dr == 0 && dc == 0) continue; msReveal(r + dr, c + dc); }
}
bool msCheckWin() { for (int r = 0; r < MS_SIZE; r++) for (int c = 0; c < MS_SIZE; c++) if (!msMine[r][c] && !msRevealed[r][c]) return false; return true; }
void drawMinesweeper() {
  lcdFillScreen(getThemeBg()); lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  int flagsUsed = 0; for (int r = 0; r < MS_SIZE; r++) for (int c = 0; c < MS_SIZE; c++) if (msFlagged[r][c]) flagsUsed++;
  char cntStr[16]; snprintf(cntStr, sizeof(cntStr), "MINES: %d", MS_MINES - flagsUsed); lcdDrawText(10, 10, cntStr, COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH - 180, 3, 80, 24, msFlagMode ? COL_YELLOW : getThemeSecondary()); lcdDrawTextCentered(LCD_WIDTH - 180, 80, 10, "FLAG", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH - 90, 3, 80, 24, COL_RED); lcdDrawTextCentered(LCD_WIDTH - 90, 80, 10, "RESET", COL_WHITE, 1);
  for (int r = 0; r < MS_SIZE; r++) for (int c = 0; c < MS_SIZE; c++) {
      int x = MS_OFF_X + c * MS_CELL, y = MS_OFF_Y + r * MS_CELL;
      if (!msRevealed[r][c]) { lcdFillRect(x, y, MS_CELL - 1, MS_CELL - 1, msFlagged[r][c] ? COL_YELLOW : getThemeSecondary()); lcdFillRect(x + 2, y + 2, MS_CELL - 5, MS_CELL - 5, msFlagged[r][c] ? 0xFD20 : getThemePrimary()); if (msFlagged[r][c]) lcdDrawTextCentered(x, MS_CELL - 1, y + 8, "F", COL_RED, 1); }
      else if (msMine[r][c]) { lcdFillRect(x, y, MS_CELL - 1, MS_CELL - 1, COL_RED); lcdFillCircle(x + MS_CELL / 2, y + MS_CELL / 2, 6, COL_BLACK); }
      else { lcdFillRect(x, y, MS_CELL - 1, MS_CELL - 1, 0x18E3); if (msAdj[r][c] > 0) { char n[2]; n[0] = '0' + msAdj[r][c]; n[1] = 0; uint16_t col = msAdj[r][c] == 1 ? COL_GREEN : msAdj[r][c] == 2 ? COL_YELLOW : COL_RED; lcdDrawTextCentered(x, MS_CELL - 1, y + 8, n, col, 1); } }
    }
  if (msGameOver) lcdDrawTextCentered(MS_OFF_X, MS_SIZE * MS_CELL, MS_OFF_Y + MS_SIZE * MS_CELL + 2, msWon ? "YOU WIN! TAP TO RESTART" : "BOOM! TAP TO RESTART", msWon ? COL_GREEN : COL_RED, 1);
  drawControlBar();
}
void handleMinesweeperTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  if (ty < 30) { if (tx >= LCD_WIDTH - 180 && tx < LCD_WIDTH - 100) { uiClick(); msFlagMode = !msFlagMode; drawMinesweeper(); return; } if (tx >= LCD_WIDTH - 90 && tx < LCD_WIDTH - 10) { uiClick(); initMinesweeper(); drawMinesweeper(); return; } return; }
  if (msGameOver) { uiClick(); initMinesweeper(); drawMinesweeper(); return; }
  int c = (tx - MS_OFF_X) / MS_CELL, r = (ty - MS_OFF_Y) / MS_CELL; if (r < 0 || r >= MS_SIZE || c < 0 || c >= MS_SIZE) return; uiClick();
  if (msFlagMode) { if (!msRevealed[r][c]) msFlagged[r][c] = !msFlagged[r][c]; }
  else { if (msFlagged[r][c]) { drawMinesweeper(); return; } if (msMine[r][c]) { msRevealed[r][c] = true; msGameOver = true; msWon = false; for (int rr = 0; rr < MS_SIZE; rr++) for (int cc = 0; cc < MS_SIZE; cc++) if (msMine[rr][cc]) msRevealed[rr][cc] = true; playError(); }
    else { msReveal(r, c); if (msCheckWin()) { msGameOver = true; msWon = true; playSuccess(); } } } drawMinesweeper();
}

// ---------- Sketch ----------
#define SKETCH_TOOLBAR_H 36
#define SKETCH_SWATCH_COUNT 8
#define SKETCH_SWATCH_W 24
#define SKETCH_SWATCH_GAP 6
#define SKETCH_SWATCH_X0 6
#define SKETCH_SWATCH_Y 6
#define SKETCH_BRUSH_X0 270
#define SKETCH_BRUSH_COUNT 4
uint16_t sketchColor = COL_WHITE; uint8_t sketchBrush = 2;
const uint16_t sketchPalette[SKETCH_SWATCH_COUNT] = { COL_WHITE, COL_RED, COL_YELLOW, COL_GREEN, 0x07FF, COL_BLUE, 0xF81F, COL_GRAY };
const uint8_t sketchBrushes[SKETCH_BRUSH_COUNT] = {1, 2, 4, 7};
void drawSketchToolbar() {
  lcdFillRect(0, 0, LCD_WIDTH, SKETCH_TOOLBAR_H, getThemeSecondary()); lcdFillRect(0, SKETCH_TOOLBAR_H - 2, LCD_WIDTH, 2, getThemePrimary());
  for (int i = 0; i < SKETCH_SWATCH_COUNT; i++) { int x = SKETCH_SWATCH_X0 + i * (SKETCH_SWATCH_W + SKETCH_SWATCH_GAP); lcdFillRect(x, SKETCH_SWATCH_Y, SKETCH_SWATCH_W, SKETCH_SWATCH_W, sketchPalette[i]);
    if (sketchPalette[i] == sketchColor) { lcdFillRect(x - 2, SKETCH_SWATCH_Y - 2, SKETCH_SWATCH_W + 4, 2, COL_YELLOW); lcdFillRect(x - 2, SKETCH_SWATCH_Y + SKETCH_SWATCH_W, SKETCH_SWATCH_W + 4, 2, COL_YELLOW); lcdFillRect(x - 2, SKETCH_SWATCH_Y - 2, 2, SKETCH_SWATCH_W + 4, COL_YELLOW); lcdFillRect(x + SKETCH_SWATCH_W, SKETCH_SWATCH_Y - 2, 2, SKETCH_SWATCH_W + 4, COL_YELLOW); } }
  for (int i = 0; i < SKETCH_BRUSH_COUNT; i++) { int cx = SKETCH_BRUSH_X0 + i * 34 + 12; int cy = SKETCH_TOOLBAR_H / 2; lcdFillCircle(cx, cy, sketchBrushes[i], COL_WHITE); if (sketchBrushes[i] == sketchBrush) lcdDrawCircle(cx, cy, sketchBrushes[i] + 4, COL_YELLOW); }
  // SAVE button
  lcdFillRect(LCD_WIDTH - 70, 4, 60, 28, COL_GREEN); lcdFillRect(LCD_WIDTH - 68, 6, 56, 24, 0x0180);
  lcdDrawTextCentered(LCD_WIDTH - 70, 60, 12, "SAVE", COL_WHITE, 1);
}
void drawSketchPad() {
  lcdFillScreen(getThemeBg()); drawSketchToolbar(); drawControlBar();
  // Allocate framebuffer in PSRAM if not already
  if (!sketchFB) {
    sketchFB = (uint16_t*)heap_caps_malloc((size_t)SKETCH_FB_W * SKETCH_FB_H * 2, MALLOC_CAP_SPIRAM);
  }
  // Fill with background color
  if (sketchFB) {
    uint16_t bg = getThemeBg();
    for (int i = 0; i < SKETCH_FB_W * SKETCH_FB_H; i++) sketchFB[i] = bg;
  }
}
// Load a saved .espimg into the sketchpad canvas (centered, clipped to fit) so it can be edited.
void openEspImgInSketchpad(const String &path) {
  uint16_t imgW, imgH; uint16_t *imgBuf = nullptr;
  if (!loadEspImg(path.c_str(), imgW, imgH, imgBuf) || !imgBuf) { playError(); return; }
  if (!sketchFB) sketchFB = (uint16_t*)heap_caps_malloc((size_t)SKETCH_FB_W * SKETCH_FB_H * 2, MALLOC_CAP_SPIRAM);
  if (!sketchFB) { free(imgBuf); playError(); return; }
  uint16_t bg = getThemeBg();
  for (int i = 0; i < SKETCH_FB_W * SKETCH_FB_H; i++) sketchFB[i] = bg;
  int cw = min((int)imgW, (int)SKETCH_FB_W);
  int ch = min((int)imgH, (int)SKETCH_FB_H);
  int ox = (SKETCH_FB_W - cw) / 2;
  int oy = (SKETCH_FB_H - ch) / 2;
  for (int y = 0; y < ch; y++) memcpy(&sketchFB[(oy + y) * SKETCH_FB_W + ox], &imgBuf[y * imgW], (size_t)cw * 2);
  free(imgBuf);
  currentMode = MODE_SKETCHPAD;
  lcdFillScreen(bg); drawSketchToolbar(); drawControlBar();
  // Blit the loaded image onto the canvas area
  lcdSetAddrWindow(0, SKETCH_TOOLBAR_H, SKETCH_FB_W - 1, SKETCH_TOOLBAR_H + SKETCH_FB_H - 1);
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_LCD_DC, HIGH);
  lcdSPI.transferBytes((uint8_t*)sketchFB, nullptr, (size_t)SKETCH_FB_W * SKETCH_FB_H * 2);
  lcdSPI.endTransaction();
  if (lcdFB) { for (int y = 0; y < SKETCH_FB_H; y++) memcpy(&lcdFB[(SKETCH_TOOLBAR_H + y) * LCD_WIDTH], &sketchFB[y * SKETCH_FB_W], (size_t)SKETCH_FB_W * 2); }
}
void handleSketchPadTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) return;
  if (ty < SKETCH_TOOLBAR_H) {
    // SAVE button
    // SAVE button — go to filename input
    if (tx >= LCD_WIDTH - 70 && tx < LCD_WIDTH - 10 && ty >= 4 && ty < 32) { uiClick(); saveContext = 1; saveReturnMode = MODE_SKETCHPAD; filenameInput = ""; currentMode = MODE_FILENAME_INPUT; drawFilenameInput(); return; }
    if (tx >= SKETCH_SWATCH_X0 && ty >= SKETCH_SWATCH_Y - 2 && ty <= SKETCH_SWATCH_Y + SKETCH_SWATCH_W + 2) { int idx = (tx - SKETCH_SWATCH_X0) / (SKETCH_SWATCH_W + SKETCH_SWATCH_GAP); if (idx >= 0 && idx < SKETCH_SWATCH_COUNT) { uiClick(); sketchColor = sketchPalette[idx]; drawSketchToolbar(); return; } }
    if (tx >= SKETCH_BRUSH_X0) { int idx = (tx - SKETCH_BRUSH_X0) / 34; if (idx >= 0 && idx < SKETCH_BRUSH_COUNT) { uiClick(); sketchBrush = sketchBrushes[idx]; drawSketchToolbar(); return; } } return;
  }
  uint8_t r = sketchBrush; lcdFillRect(tx - r, ty - r, r * 2 + 1, r * 2 + 1, sketchColor);
  // Also write to framebuffer
  if (sketchFB) {
    int fbX0 = max(0, (int)tx - r);
    int fbY0 = max(0, (int)ty - r - SKETCH_TOOLBAR_H);
    int fbX1 = min((int)SKETCH_FB_W - 1, (int)tx + r);
    int fbY1 = min((int)SKETCH_FB_H - 1, (int)ty + r - SKETCH_TOOLBAR_H);
    for (int y = fbY0; y <= fbY1; y++) {
      for (int x = fbX0; x <= fbX1; x++) {
        sketchFB[y * SKETCH_FB_W + x] = sketchColor;
      }
    }
  }
}

// ---------- Notepad App ----------
void drawNotepad() {
  lcdFillScreen(getThemeBg()); lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary()); lcdDrawText(10, 10, "NOTEPAD", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH - 120, 5, 50, 20, COL_GREEN); lcdDrawTextCentered(LCD_WIDTH - 120, 50, 10, "SAVE", COL_WHITE, 1);
  lcdFillRect(LCD_WIDTH - 60, 5, 50, 20, COL_RED); lcdDrawTextCentered(LCD_WIDTH - 60, 50, 10, "CLEAR", COL_WHITE, 1);
  lcdFillRect(10, 35, LCD_WIDTH - 20, 70, 0x1080); drawWrappedText(15, 40, LCD_WIDTH - 40, notepadText.c_str(), COL_WHITE, 1);
  int kbStartY = 110; int keyW = 42; int keyH = 42; int startX = 30;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 10; c++) {
      const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      int bx = startX + c * keyW; int by = kbStartY + r * keyH;
      if (strcmp(label, "SPACE") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "SPACE") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "SPACE") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, 0x2104); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "SPACE", COL_GRAY, 1);
        } continue;
      }
      if (strcmp(label, "OK") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "OK") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "OK") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, getThemePrimary()); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "ENTER", COL_WHITE, 1);
        } continue;
      }
      uint16_t btnCol = getThemeSecondary();
      if (strcmp(label, "DEL") == 0 || strcmp(label, "SHIFT") == 0 || strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) btnCol = getThemePrimary();
      lcdFillRect(bx, by, keyW, keyH, btnCol); lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemeBg()); char disp[8]; strncpy(disp, label, 7); disp[7] = 0;
      if (!shiftActive && disp[0] >= 'A' && disp[0] <= 'Z' && strlen(disp) == 1) disp[0] += 32;
      lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, disp, COL_WHITE, 1);
      if (strcmp(label, "SHIFT") == 0 && shiftActive) { lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemePrimary()); lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, "SHIFT", COL_WHITE, 1); }
    }
  }
  drawControlBar();
}
bool handleNotepadTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return true; }
  if (tx >= LCD_WIDTH - 120 && tx < LCD_WIDTH - 70 && ty >= 5 && ty < 25) { uiClick(); saveContext = 0; saveReturnMode = MODE_NOTEPAD; filenameInput = ""; currentMode = MODE_FILENAME_INPUT; drawFilenameInput(); return true; }
  if (tx >= LCD_WIDTH - 60 && tx < LCD_WIDTH - 10 && ty >= 5 && ty < 25) { uiClick(); notepadText = ""; drawNotepad(); return true; }
  int kbStartY = 110; int keyW = 42; int keyH = 42; int startX = 30;
  if (ty >= kbStartY) { int r = (ty - kbStartY) / keyH; int c = (tx - startX) / keyW;
    if (r >= 0 && r < 4 && c >= 0 && c < 10 && tx >= startX) { playKeyBeep(); const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      if (strcmp(label, "DEL") == 0) { if (notepadText.length() > 0) notepadText.remove(notepadText.length() - 1); }
      else if (strcmp(label, "SHIFT") == 0) { shiftActive = !shiftActive; }
      else if (strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) { isNumLayout = !isNumLayout; }
      else if (strcmp(label, "SPACE") == 0) { notepadText += ' '; }
      else if (strcmp(label, "OK") == 0) { notepadText += '\n'; }
      else { char ch = label[0]; if (!shiftActive && ch >= 'A' && ch <= 'Z') ch += 32; notepadText += ch; if (shiftActive) shiftActive = false; }
      drawNotepad(); return true; } } return false;
}

// ---------- Camera ----------
#define CAM_SHUTTER_CX 445
#define CAM_SHUTTER_CY 160
#define CAM_SHUTTER_R  28
void initCamera() {
  if (!settingCameraAccess) { showMessageBox(MSG_CRIT, "ACCESS DENIED", "CAMERA ACCESS IS", "DISABLED IN SETTINGS", MODE_LAUNCHER); return; }
  camSetPowerDown(false); delay(10); camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1; config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3; config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5; config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK; config.pin_pclk = CAM_PIN_PCLK; config.pin_vsync = CAM_PIN_VSYNC; config.pin_href = CAM_PIN_HREF; config.pin_sccb_sda = CAM_PIN_SIOD; config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN; config.pin_reset = CAM_PIN_RESET; config.xclk_freq_hz = 10000000; config.pixel_format = PIXFORMAT_RGB565; config.frame_size = FRAMESIZE_QVGA; config.jpeg_quality = 12; config.fb_count = 1; config.fb_location = CAMERA_FB_IN_PSRAM; config.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // FIX: Re-init Wire even on failure, since camera init may have touched the shared I2C bus
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); ft6336AutoDetect();
    Serial.printf("Camera init failed: 0x%x\n", err); char codeStr[24]; snprintf(codeStr, sizeof(codeStr), "CODE: 0X%X", err); showMessageBox(MSG_CRIT, "CAMERA ERROR", "CAMERA INIT FAILED", codeStr, MODE_LAUNCHER); return;
  }
  Serial.println("Camera initialized!");
  // FIX: Camera SCCB and touch share I2C bus (IO7/IO8). Re-init Wire so touch works after camera init.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  ft6336AutoDetect();
  lcdFillScreen(COL_BLACK);
}
void drawDiagnosticOverlay() {
  lcdFillRect(0, 0, LCD_WIDTH, 40, 0x0000);
  lcdFillRect(10, 10, 100, 10, 0x0F00); lcdDrawText(12, 11, "PSRAM: OK", COL_WHITE, 1);
  lcdFillRect(120, 10, 100, 10, 0x0F00); lcdDrawText(122, 11, "CAM: OK", COL_WHITE, 1);
  lcdFillRect(230, 10, 100, 10, 0x0F00); lcdDrawText(232, 11, "QVGA", COL_WHITE, 1);
  lcdFillRect(350, 10, 80, 20, getThemePrimary()); char rotStr[16]; snprintf(rotStr, sizeof(rotStr), "ROT:%d", cameraRotation); lcdDrawTextCentered(350, 80, 14, rotStr, COL_WHITE, 1);
}
static uint16_t *camDispBuf = nullptr;
void prepareCameraFrame(camera_fb_t *fb, int rotation, int &outW, int &outH) {
  const int srcW = 320, srcH = 240;
  if (!camDispBuf) { camDispBuf = (uint16_t *)heap_caps_malloc((size_t)srcW * srcH * 2, MALLOC_CAP_SPIRAM); if (!camDispBuf) { outW = 0; outH = 0; return; } }
  const uint16_t *pixels = (const uint16_t *)fb->buf;
  if (rotation == 0) { memcpy(camDispBuf, pixels, (size_t)srcW * srcH * 2); outW = srcW; outH = srcH; return; }
  if (rotation == 180) { for (int y = 0; y < srcH; y++) for (int x = 0; x < srcW; x++) camDispBuf[y * srcW + x] = pixels[(srcH - 1 - y) * srcW + (srcW - 1 - x)]; outW = srcW; outH = srcH; return; }
  const int rotW = srcH; const int rotH = srcW; const int cropH = srcH; const int cropTop = (rotH - cropH) / 2;
  if (rotation == 90) { for (int i = 0; i < cropH; i++) { int ri = i + cropTop; for (int j = 0; j < rotW; j++) camDispBuf[i * rotW + j] = pixels[(srcH - 1 - j) * srcW + ri]; } }
  else { for (int i = 0; i < cropH; i++) { int ri = i + cropTop; for (int j = 0; j < rotW; j++) camDispBuf[i * rotW + j] = pixels[j * srcW + (srcW - 1 - ri)]; } }
  outW = rotW; outH = cropH;
}
void drawShutterButton() {
  bool flashing = millis() < camShutterFlashUntil;
  lcdFillCircle(CAM_SHUTTER_CX, CAM_SHUTTER_CY, CAM_SHUTTER_R, flashing ? COL_YELLOW : COL_WHITE);
  lcdDrawCircle(CAM_SHUTTER_CX, CAM_SHUTTER_CY, CAM_SHUTTER_R, COL_DKGRAY);
  lcdDrawCircle(CAM_SHUTTER_CX, CAM_SHUTTER_CY, CAM_SHUTTER_R - 1, COL_DKGRAY);
  lcdFillCircle(CAM_SHUTTER_CX, CAM_SHUTTER_CY, CAM_SHUTTER_R - 6, flashing ? COL_WHITE : getThemePrimary());
}
void captureCameraPhoto() {
  if (camLastDispW == 0 || camLastDispH == 0 || !camDispBuf) { playError(); char psramMsg[28]; snprintf(psramMsg, sizeof(psramMsg), "PSRAM SIZE: %d", ESP.getPsramSize()); showMessageBox(MSG_CRIT, "CAPTURE FAILED", "NO PSRAM BUFFER", psramMsg, currentMode); return; }
  camShutterFlashUntil = millis() + 150;
  drawShutterButton();
  uiClick();
  if (!sdCardMounted) { playError(); showMessageBox(MSG_WARN, "CAPTURE FAILED", "SD CARD NOT", "MOUNTED", currentMode); return; }
  prefs.begin("wave_os", false);
  camPhotoCount = prefs.getInt("cam_cnt", 0);
  char fname[32]; snprintf(fname, sizeof(fname), "/photo%d.espimg", camPhotoCount);
  prefs.putInt("cam_cnt", camPhotoCount + 1);
  prefs.end();
  if (saveEspImgFromBuffer(fname, camDispBuf, camLastDispW, camLastDispH)) playSuccess(); else { playError(); showMessageBox(MSG_WARN, "CAPTURE FAILED", "SD WRITE ERROR", "SEE SERIAL LOG", currentMode); }
}
void updateCamera() {
  camera_fb_t *fb = esp_camera_fb_get(); if (!fb) return;
  int dispW, dispH; prepareCameraFrame(fb, cameraRotation, dispW, dispH);
  if (dispW == 0 || dispH == 0) { esp_camera_fb_return(fb); return; }
  camLastDispW = dispW; camLastDispH = dispH;
  int areaX0 = (cameraMode == CAM_MODE_DIAGNOSTIC) ? 0 : 80; int areaW = 320;
  int x0 = areaX0 + (areaW - dispW) / 2; int y0 = 40;
  lcdSetAddrWindow(x0, y0, x0 + dispW - 1, y0 + dispH - 1);
  lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0)); digitalWrite(PIN_LCD_DC, HIGH);
  lcdSPI.transferBytes((uint8_t *)camDispBuf, nullptr, (size_t)dispW * dispH * 2); lcdSPI.endTransaction();
  if (cameraMode == CAM_MODE_DIAGNOSTIC) drawDiagnosticOverlay();
  drawShutterButton();
  esp_camera_fb_return(fb);
  // FIX: Camera SCCB traffic can knock the touch controller off the shared I2C bus over time;
  // periodically re-sync so touch keeps responding during a live preview.
  static uint8_t camTouchResyncCounter = 0;
  if (++camTouchResyncCounter >= 30) { camTouchResyncCounter = 0; Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); ft6336AutoDetect(); }
}
void handleCameraRotationTouch(uint16_t tx, uint16_t ty) {
  // Shutter button takes priority over rotate-on-tap
  int dx = (int)tx - CAM_SHUTTER_CX, dy = (int)ty - CAM_SHUTTER_CY;
  if (dx * dx + dy * dy <= CAM_SHUTTER_R * CAM_SHUTTER_R) { captureCameraPhoto(); return; }
  // FIX: Control bar is handled by the main loop dispatch, but we also handle rotation here.
  if (cameraMode == CAM_MODE_DIAGNOSTIC && tx >= 350 && tx < 430 && ty >= 10 && ty < 30) { uiClick(); cameraRotation = (cameraRotation + 90) % 360; saveSettings(); }
  // FIX: Allow tapping the camera view area to rotate in any mode
  if (ty >= 40 && ty < BAR_Y && tx >= 80 && tx < 400) { uiClick(); cameraRotation = (cameraRotation + 90) % 360; saveSettings(); }
}

// ---------- IP Explorer ----------
void drawIpExplorer() {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  lcdDrawText(10, 10, "IP EXPLORER", COL_WHITE, 1);
  
  if (!wifiConnected) {
    lcdDrawTextCentered(0, LCD_WIDTH, 100, "CONNECT TO WIFI FIRST", COL_RED, 1);
  } else {
    lcdFillRect(30, 40, 200, 30, getThemeSecondary());
    lcdDrawTextCentered(30, 200, 48, ipScanning ? "STOP SCAN" : "SCAN NETWORK", COL_WHITE, 1);
    
    if (ipScanning) {
      lcdDrawTextCentered(0, LCD_WIDTH, 80, "SCANNING... (EXIT OK, runs in background)", COL_YELLOW, 1);
      // Draw progress bar background
      int pbX = 60, pbY = 95, pbW = 360, pbH = 20;
      int fillW = (int)((float)ipScanProgress / 253.0 * pbW);
      lcdFillRect(pbX, pbY, pbW, pbH, 0x2104);
      lcdFillRect(pbX, pbY, fillW, pbH, COL_MINT);
      lcdFillRect(pbX, pbY + pbH, pbW, 2, COL_DKGRAY);
      char pctStr[16]; snprintf(pctStr, sizeof(pctStr), "%d%%", (int)((float)ipScanProgress/253.0*100));
      lcdDrawTextCentered(pbX, pbW, pbY + 6, pctStr, COL_WHITE, 1);
      char foundStr[16]; snprintf(foundStr, sizeof(foundStr), "FOUND: %d", ipCount);
      lcdDrawTextCentered(0, LCD_WIDTH, pbY + pbH + 8, foundStr, COL_GREEN, 1);
    } else {
      lcdDrawText(10, 80, "FOUND DEVICES:", COL_GRAY, 1);
      int startY = 95;
      int rowH = 20;
      for (int i = 0; i < ipCount && i < 10; i++) {
        lcdFillRect(20, startY + i*rowH, LCD_WIDTH-40, rowH-2, 0x2104);
        lcdDrawText(30, startY + i*rowH + 6, ipList[i].c_str(), COL_WHITE, 1);
      }
      if (ipCount == 0 && !ipScanning) {
        lcdDrawTextCentered(0, LCD_WIDTH, 120, "TAP SCAN TO START", COL_GRAY, 1);
      }
    }
  }
  drawControlBar();
}

void handleIpExplorerTouch(uint16_t tx, uint16_t ty) {
  // Exiting via the control bar (or START menu, etc.) no longer cancels an
  // in-progress scan - the scan keeps stepping in the background from
  // cpuTask's loop regardless of currentMode, so the user is free to go do
  // something else while it finishes.
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  
  if (!wifiConnected) return;
  
  // Scan / Stop Button (same button toggles between the two states)
  if (tx >= 30 && tx < 230 && ty >= 40 && ty < 70) {
    uiClick();
    if (ipScanning) {
      // Explicit stop request
      ipScanCancelled = true;
      ipScanFinish(true);
    } else {
      ipCount = 0;
      ipScanProgress = 0;
      ipScanIndex = 1;
      ipScanBaseIP = WiFi.localIP();
      ipScanCancelled = false;
      ipScanning = true;
      ipScanLastStepTime = 0; // let the very next cpuTask pass step immediately
      drawIpExplorer();
    }
    return;
  }
  
  // Click on IP
  if (!ipScanning) {
    int startY = 95;
    int rowH = 20;
    for (int i = 0; i < ipCount && i < 10; i++) {
      if (ty >= startY + i*rowH && ty < startY + (i+1)*rowH) {
        uiClick();
        ipViewerUrl = ipList[i];
        currentMode = MODE_IP_VIEWER;
        drawIpViewer();
        return;
      }
    }
  }
}

// ---------- Background scan engine ----------
// Called from cpuTask's loop, throttled to one probe roughly every
// IP_SCAN_STEP_INTERVAL_MS. This is "the scanning" - the only piece that
// stays tied to Core 0 network I/O. Everything else (progress bar redraw,
// touch handling, every other app) is dispatched through the GPU queue as
// normal and is never blocked by it, so backing out of IP Explorer mid-scan
// leaves the rest of the OS fully responsive.
void ipScanStep() {
  int i = ipScanIndex;
  IPAddress target(ipScanBaseIP[0], ipScanBaseIP[1], ipScanBaseIP[2], i);
  HTTPClient http;
  http.begin("http://" + target.toString());
  // Bound BOTH the TCP connect phase and the response-read phase - the
  // Arduino HTTPClient's default connect timeout (several seconds) is what
  // was making the app feel un-exitable: setTimeout() alone only covers the
  // read, not the initial connect to a dead/non-existent host.
  http.setConnectTimeout(150);
  http.setTimeout(150);
  int httpCode = http.GET();
  if (httpCode > 0) {
    ipList[ipCount++] = "http://" + target.toString();
  }
  http.end();
  ipScanProgress = i;
  ipScanIndex++;

  // Only push a progress-bar redraw to the GPU if IP Explorer is actually
  // the screen currently on display - saves GPU cycles for whatever app the
  // user switched to.
  if (currentMode == MODE_IP_EXPLORER) {
    GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawIpExplorer};
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  }

  if (ipScanIndex >= 255 || ipCount >= 20) {
    ipScanFinish(false);
  }
}

// Ends the scan (naturally finished, explicitly stopped, WiFi dropped, or
// the power button was pressed) and always surfaces a notification with a
// VIEW button - even if IP Explorer isn't the current screen - so results
// found so far are never silently lost.
void ipScanFinish(bool cancelled) {
  ipScanning = false;
  char resultMsg[48];
  if (cancelled) {
    snprintf(resultMsg, sizeof(resultMsg), "SCAN STOPPED - %d FOUND", ipCount);
  } else {
    snprintf(resultMsg, sizeof(resultMsg), "SCAN COMPLETE - %d FOUND", ipCount);
  }
  showNotificationWithView(resultMsg, MODE_IP_EXPLORER);
  if (currentMode == MODE_IP_EXPLORER) {
    GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawIpExplorer};
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  }
}

void drawIpViewer() {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  lcdDrawText(10, 10, "WEB VIEWER", COL_WHITE, 1);
  lcdDrawText(10, 10, ipViewerUrl.c_str(), COL_YELLOW, 1);
  
  lcdDrawTextCentered(0, LCD_WIDTH, 100, "LOADING...", COL_WHITE, 1);
  
  HTTPClient http;
  http.begin(ipViewerUrl);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    ipViewerContent = http.getString();
    // Truncate for display
    if (ipViewerContent.length() > 500) {
      ipViewerContent = ipViewerContent.substring(0, 500) + "...";
    }
  } else {
    ipViewerContent = "FAILED TO LOAD CONTENT";
  }
  http.end();
  
  lcdFillRect(10, 40, LCD_WIDTH-20, LCD_HEIGHT-80, 0x1080);
  drawWrappedText(15, 45, LCD_WIDTH-40, ipViewerContent.c_str(), COL_WHITE, 1);
  drawControlBar();
}

// ---------- Filename Input ----------
void drawFilenameInput() {
  lcdFillScreen(getThemeBg());
  lcdFillRect(0, 0, LCD_WIDTH, 36, getThemePrimary());
  const char* title = saveContext == 2 ? "RENAME FILE" : "SAVE AS";
  lcdDrawText(10, 12, title, COL_WHITE, 2);
  const char* ext = saveContext == 0 ? ".esptxt" : (saveContext == 1 ? ".espimg" : "");
  const char* label1 = saveContext == 0 ? "TEXT FILE (.esptxt)" : (saveContext == 1 ? "PAINTING (.espimg)" : "ENTER NEW NAME");
  lcdDrawTextCentered(0, LCD_WIDTH, 45, label1, COL_GRAY, 1);
  // Input field
  lcdFillRect(40, 65, LCD_WIDTH - 80, 25, 0x2104);
  lcdFillRect(42, 67, LCD_WIDTH - 84, 21, COL_BLACK);
  char typed[33]; int len = filenameInput.length(); int n = min(len, 30);
  memcpy(typed, filenameInput.c_str(), n); typed[n] = '\0';
  strcat(typed, "_"); // cursor
  lcdDrawText(50, 73, typed, COL_WHITE, 1);
  // Extension hint
  lcdDrawTextCentered(0, LCD_WIDTH, 95, saveContext == 2 ? "Keep extension or type new one" : (saveContext == 0 ? "Extension auto-added: .esptxt" : "Extension auto-added: .espimg"), COL_GRAY, 1);
  // Keyboard
  int kbStartY = 115; int keyW = 42; int keyH = 42; int startX = 30;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 10; c++) {
      const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      int bx = startX + c * keyW; int by = kbStartY + r * keyH;
      if (strcmp(label, "SPACE") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "SPACE") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "SPACE") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, 0x2104); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, "_", COL_GRAY, 1);
        } continue;
      }
      if (strcmp(label, "OK") == 0) {
        if (c == 0 || strcmp(isNumLayout ? kbNum[r][c-1] : kbAlpha[r][c-1], "OK") != 0) {
          int span = 1; while (c + span < 10 && strcmp(isNumLayout ? kbNum[r][c+span] : kbAlpha[r][c+span], "OK") == 0) span++;
          int wideW = span * keyW;
          lcdFillRect(bx, by, wideW, keyH, COL_GREEN); lcdFillRect(bx+2, by+2, wideW-4, keyH-4, getThemeBg());
          lcdDrawTextCentered(bx, wideW, by + keyH/2 - 4, saveContext == 2 ? "RENAME" : "SAVE", COL_WHITE, 1);
        } continue;
      }
      uint16_t btnCol = getThemeSecondary();
      if (strcmp(label, "DEL") == 0 || strcmp(label, "SHIFT") == 0 || strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) btnCol = getThemePrimary();
      lcdFillRect(bx, by, keyW, keyH, btnCol); lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemeBg());
      char disp[8]; strncpy(disp, label, 7); disp[7] = 0;
      if (!shiftActive && disp[0] >= 'A' && disp[0] <= 'Z' && strlen(disp) == 1) disp[0] += 32;
      lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, disp, COL_WHITE, 1);
      if (strcmp(label, "SHIFT") == 0 && shiftActive) { lcdFillRect(bx+2, by+2, keyW-4, keyH-4, getThemePrimary()); lcdDrawTextCentered(bx, keyW, by + keyH/2 - 4, "SHIFT", COL_WHITE, 1); }
    }
  }
  drawControlBar();
}
bool handleFilenameInputTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return true; }
  int kbStartY = 115; int keyW = 42; int keyH = 42; int startX = 30;
  if (ty >= kbStartY) { int r = (ty - kbStartY) / keyH; int c = (tx - startX) / keyW;
    if (r >= 0 && r < 4 && c >= 0 && c < 10 && tx >= startX) { playKeyBeep(); const char* label = isNumLayout ? kbNum[r][c] : kbAlpha[r][c];
      if (strcmp(label, "DEL") == 0) { if (filenameInput.length() > 0) filenameInput.remove(filenameInput.length() - 1); }
      else if (strcmp(label, "SHIFT") == 0) { shiftActive = !shiftActive; }
      else if (strcmp(label, "NUM") == 0 || strcmp(label, "ABC") == 0) { isNumLayout = !isNumLayout; }
      else if (strcmp(label, "SPACE") == 0) { if (filenameInput.length() < 24) filenameInput += '_'; }
      else if (strcmp(label, "OK") == 0) {
        // Save or rename the file
        if (filenameInput.length() == 0) { playError(); return true; }
        if (saveContext == 0) {
          // Save notepad text
          char fname[40]; snprintf(fname, sizeof(fname), "/%s.esptxt", filenameInput.c_str());
          if (saveEspTxt(fname, notepadText)) { playSuccess(); refreshFileList(); } else playError();
        } else if (saveContext == 1) {
          // Save sketchpad image
          char fname[40]; snprintf(fname, sizeof(fname), "/%s.espimg", filenameInput.c_str());
          if (sketchFB) { if (saveEspImgFromBuffer(fname, sketchFB, SKETCH_FB_W, SKETCH_FB_H)) { playSuccess(); refreshFileList(); } else { playError(); showMessageBox(MSG_WARN, "SAVE FAILED", "SD WRITE ERROR", sdCardMounted ? "SEE SERIAL LOG" : "SD NOT MOUNTED", saveReturnMode); } }
          else { playError(); char psramMsg[28]; snprintf(psramMsg, sizeof(psramMsg), "PSRAM SIZE: %d", ESP.getPsramSize()); showMessageBox(MSG_CRIT, "SAVE FAILED", "NO PSRAM BUFFER", psramMsg, saveReturnMode); }
        } else if (saveContext == 2) {
          // Rename file
          String newName = filenameInput;
          // Preserve extension if not typed
          if (newName.indexOf('.') < 0) {
            int dotIdx = renameTarget.lastIndexOf('.');
            if (dotIdx >= 0) newName += renameTarget.substring(dotIdx);
          }
          String newPath = "/" + newName;
          if (SD_MMC.rename(renameTarget.c_str(), newPath.c_str())) {
            playSuccess(); refreshFileList();
          } else {
            playError();
          }
        }
        currentMode = saveReturnMode;
        if (saveReturnMode == MODE_NOTEPAD) drawNotepad();
        else if (saveReturnMode == MODE_SKETCHPAD) drawSketchPad();
        else if (saveReturnMode == MODE_FILE_BROWSER) drawFileBrowser();
        return true;
      }
      else { char ch = label[0]; if (!shiftActive && ch >= 'A' && ch <= 'Z') ch += 32; if (filenameInput.length() < 24) filenameInput += ch; if (shiftActive) shiftActive = false; }
      drawFilenameInput(); return true;
    }
  }
  return false;
}

// ---------- SD Confirm Dialog ----------
void drawSdConfirm() {
  drawDialogShell(RDLG_X, RDLG_Y, RDLG_W, RDLG_H, "CLEAR ALL FILES", COL_RED);
  // Question icon
  drawQuestionIcon(RDLG_X + 50, RDLG_Y + 80, 20);
  // Warning text
  lcdDrawText(RDLG_X + 90, RDLG_Y + 60, "ALL FILES ON THE SD", COL_BLACK, 1);
  lcdDrawText(RDLG_X + 90, RDLG_Y + 76, "CARD WILL BE ERASED.", COL_BLACK, 1);
  lcdDrawText(RDLG_X + 90, RDLG_Y + 92, "ARE YOU SURE?", COL_RED, 1);
  // YES / NO buttons
  int btnW = 120, btnH = 40, gap = 20;
  int totalW = btnW * 2 + gap;
  int bx0 = RDLG_X + (RDLG_W - totalW) / 2;
  int by = RD_BTN_Y;
  // YES
  lcdFillRect(bx0, by, btnW, btnH, COL_BLACK); lcdFillRect(bx0 + 2, by + 2, btnW - 4, btnH - 4, COL_RED);
  lcdDrawTextCentered(bx0, btnW, by + btnH / 2 - 4, "YES", COL_WHITE, 2);
  // NO
  lcdFillRect(bx0 + btnW + gap, by, btnW, btnH, COL_BLACK); lcdFillRect(bx0 + btnW + gap + 2, by + 2, btnW - 4, btnH - 4, COL_GREEN);
  lcdDrawTextCentered(bx0 + btnW + gap, btnW, by + btnH / 2 - 4, "NO", COL_WHITE, 2);
}
void handleSdConfirmTouch(uint16_t tx, uint16_t ty) {
  // Close X
  int closeX = RDLG_X + RDLG_W - 20, closeY = RDLG_Y + 6;
  if (tx >= closeX - 6 && tx < closeX + 20 && ty >= closeY - 6 && ty < closeY + 20) {
    uiClick(); currentMode = MODE_FILE_BROWSER; drawFileBrowser(); return;
  }
  int btnW = 120, btnH = 40, gap = 20;
  int totalW = btnW * 2 + gap;
  int bx0 = RDLG_X + (RDLG_W - totalW) / 2;
  int by = RD_BTN_Y;
  if (ty >= by && ty < by + btnH) {
    if (tx >= bx0 && tx < bx0 + btnW) { // YES
      uiClick();
      formatSdCard();
      currentMode = MODE_FILE_BROWSER;
      drawFileBrowser();
      return;
    }
    if (tx >= bx0 + btnW + gap && tx < bx0 + btnW + gap + btnW) { // NO
      uiClick();
      currentMode = MODE_FILE_BROWSER;
      drawFileBrowser();
      return;
    }
  }
}

// ---------- File Browser ----------
void drawFileBrowser() {
  lcdFillScreen(getThemeBg());
  // Header with SD card info
  lcdFillRect(0, 0, LCD_WIDTH, FB_HEADER_H, getThemePrimary());
  lcdDrawText(8, 8, "SD CARD", COL_WHITE, 2);
  if (sdCardMounted) {
    uint64_t total = SD_MMC.totalBytes();
    uint64_t used = SD_MMC.usedBytes();
    char info[48];
    snprintf(info, sizeof(info), "%lluMB/%lluMB", used / 1048576, total / 1048576);
    lcdDrawText(140, 14, info, COL_YELLOW, 1);
    // Mini usage bar
    int barW = 80; int barX = LCD_WIDTH - barW - 10; int barY = 16;
    lcdFillRect(barX, barY, barW, 8, 0x2104);
    int fillW = total > 0 ? (int)(barW * used / total) : 0;
    lcdFillRect(barX, barY, fillW, 8, COL_MINT);
  } else {
    lcdDrawText(140, 14, "NOT MOUNTED", COL_RED, 1);
  }
  // Bottom action bar
  int btnY = BAR_Y - FB_BOTTOM_H + 4;
  lcdFillRect(0, BAR_Y - FB_BOTTOM_H, LCD_WIDTH, FB_BOTTOM_H, 0x1080);
  // FORMAT button
  lcdFillRect(10, btnY, 90, 22, COL_RED); lcdFillRect(12, btnY+2, 86, 18, getThemeBg());
  lcdDrawTextCentered(10, 90, btnY + 7, "CLEAR ALL", COL_RED, 1);
  // REFRESH button
  lcdFillRect(110, btnY, 90, 22, COL_BLUE); lcdFillRect(112, btnY+2, 86, 18, getThemeBg());
  lcdDrawTextCentered(110, 90, btnY + 7, "REFRESH", COL_WHITE, 1);
  // Refresh file list before drawing
  if (sdCardMounted) refreshFileList();
  // Scroll indicator
  if (fileCount > FB_MAX_ROWS) {
    lcdFillRect(LCD_WIDTH - 20, FB_LIST_Y0, 18, FB_LIST_Y1 - FB_LIST_Y0, 0x1080);
    int scrollH = FB_LIST_Y1 - FB_LIST_Y0;
    int thumbH = max(10, scrollH * FB_MAX_ROWS / fileCount);
    int thumbY = FB_LIST_Y0 + (scrollH - thumbH) * fileScrollOffset / max(1, fileCount - FB_MAX_ROWS);
    lcdFillRect(LCD_WIDTH - 18, thumbY, 14, thumbH, COL_MINT);
  }
  if (!sdCardMounted) {
    lcdDrawTextCentered(0, LCD_WIDTH, 100, "NO SD CARD", COL_RED, 2);
    lcdDrawTextCentered(0, LCD_WIDTH, 130, "Insert FAT32 formatted SD card", COL_GRAY, 1);
    drawControlBar();
    return;
  }
  if (fileCount == 0) {
    lcdDrawTextCentered(0, LCD_WIDTH, 100, "NO FILES FOUND", COL_GRAY, 1);
    lcdDrawTextCentered(0, LCD_WIDTH, 120, "Save from Notepad or Sketchpad", COL_GRAY, 1);
  } else {
    for (int i = 0; i < FB_MAX_ROWS && (i + fileScrollOffset) < fileCount; i++) {
      int idx = i + fileScrollOffset;
      int rowY = FB_LIST_Y0 + i * FB_ROW_H;
      // Alternating row bg
      uint16_t rowBg = (i % 2 == 0) ? 0x1080 : 0x0840;
      lcdFillRect(0, rowY, LCD_WIDTH - 22, FB_ROW_H - 1, rowBg);
      // File type icon
      uint16_t iconCol = COL_GRAY;
      if (fileList[idx].endsWith(".esptxt")) iconCol = COL_GREEN;
      else if (fileList[idx].endsWith(".espimg")) iconCol = COL_BLUE;
      else if (fileList[idx].endsWith(".txt")) iconCol = COL_GREEN;
      else if (fileList[idx].endsWith(".jpg") || fileList[idx].endsWith(".png")) iconCol = COL_BLUE;
      lcdFillRect(4, rowY + 4, 4, FB_ROW_H - 8, iconCol);
      // File name (truncate if too long)
      char nameBuf[40]; strncpy(nameBuf, fileList[idx].c_str(), 39); nameBuf[39] = 0;
      if (fileList[idx].length() > 30) { nameBuf[28] = '.'; nameBuf[29] = '.'; nameBuf[30] = 0; }
      lcdDrawText(12, rowY + 7, nameBuf, COL_WHITE, 1);
      // File size on right
      lcdDrawText(LCD_WIDTH - 80, rowY + 7, fileSizes[idx].c_str(), COL_GRAY, 1);
    }
    // Context menu overlay
    if (fileMenuIndex >= 0 && fileMenuIndex < fileCount) {
      // Dim background
      lcdFillRect(0, 0, LCD_WIDTH, BAR_Y, 0x4208);
      // Menu box
      int mw = 200, mh = 130;
      int mx = (LCD_WIDTH - mw) / 2, my = (BAR_Y - mh) / 2;
      lcdFillRect(mx, my, mw, mh, getThemeBg());
      lcdFillRect(mx, my, mw, 2, getThemePrimary());
      lcdFillRect(mx, my, 2, mh, getThemePrimary());
      lcdFillRect(mx + mw - 2, my, 2, mh, getThemePrimary());
      lcdFillRect(mx, my + mh - 2, mw, 2, getThemePrimary());
      // Title
      lcdFillRect(mx, my, mw, 22, getThemePrimary());
      char titleBuf[32]; strncpy(titleBuf, fileList[fileMenuIndex].c_str(), 31); titleBuf[31] = 0;
      lcdDrawTextCentered(mx, mw, my + 7, titleBuf, COL_WHITE, 1);
      // Buttons — always 4 rows: OPEN(0), RENAME(1), DELETE(2), CANCEL(3)
      int by = my + 30; int bw = mw - 20; int bh = 22;
      bool canOpen = fileList[fileMenuIndex].endsWith(".esptxt") || fileList[fileMenuIndex].endsWith(".espimg");
      // OPEN
      lcdFillRect(mx+10, by, bw, bh, canOpen ? COL_MINT : COL_GRAY); lcdFillRect(mx+12, by+2, bw-4, bh-4, getThemeBg()); lcdDrawTextCentered(mx+10, bw, by+7, canOpen ? "OPEN" : "OPEN (N/A)", canOpen ? COL_WHITE : COL_GRAY, 1);
      by += 26;
      // RENAME
      lcdFillRect(mx+10, by, bw, bh, COL_YELLOW); lcdFillRect(mx+12, by+2, bw-4, bh-4, getThemeBg()); lcdDrawTextCentered(mx+10, bw, by+7, "RENAME", COL_WHITE, 1);
      by += 26;
      // DELETE
      lcdFillRect(mx+10, by, bw, bh, COL_RED); lcdFillRect(mx+12, by+2, bw-4, bh-4, getThemeBg()); lcdDrawTextCentered(mx+10, bw, by+7, "DELETE", COL_WHITE, 1);
      by += 26;
      // CANCEL
      lcdFillRect(mx+10, by, bw, bh, COL_GRAY); lcdFillRect(mx+12, by+2, bw-4, bh-4, getThemeBg()); lcdDrawTextCentered(mx+10, bw, by+7, "CANCEL", COL_WHITE, 1);
    }
  }
  drawControlBar();
}
void handleFileBrowserTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  // If viewing an image, check EDIT button first, otherwise any tap returns to file list
  if (fileViewerActive) {
    if (tx >= LCD_WIDTH - 190 && tx < LCD_WIDTH - 90 && ty >= 3 && ty < 27) {
      uiClick(); fileViewerActive = false;
      openEspImgInSketchpad(fileViewerPath);
      return;
    }
    fileViewerActive = false; uiClick(); drawFileBrowser(); return;
  }
  // Bottom action bar
  if (ty >= BAR_Y - FB_BOTTOM_H) {
    int btnY = BAR_Y - FB_BOTTOM_H + 4;
    if (ty >= btnY && ty < btnY + 22) {
      if (tx >= 10 && tx < 100) { // CLEAR ALL
        uiClick();
        currentMode = MODE_SD_CONFIRM;
        drawSdConfirm();
        return;
      }
      if (tx >= 110 && tx < 200) { // REFRESH
        uiClick(); refreshFileList(); drawFileBrowser(); return;
      }
    }
    return;
  }
  if (!sdCardMounted) return;
  // Context menu handling
  if (fileMenuIndex >= 0) {
    int mw = 200, mh = 130;
    int mx = (LCD_WIDTH - mw) / 2, my = (BAR_Y - mh) / 2;
    if (tx >= mx && tx < mx + mw && ty >= my && ty < my + mh) {
      int by = my + 30; int bw = mw - 20; int bh = 22;
      bool canOpen = fileList[fileMenuIndex].endsWith(".esptxt") || fileList[fileMenuIndex].endsWith(".espimg");
      int btnIdx = (ty - (my + 30)) / 26;
      if (btnIdx < 0 || btnIdx > 3) return;
      // Fixed indices: OPEN=0, RENAME=1, DELETE=2, CANCEL=3
      if (btnIdx == 0) { // OPEN
        if (!canOpen) { playError(); return; } // Can't open
        uiClick();
        String path = "/" + fileList[fileMenuIndex];
        int fmi = fileMenuIndex; fileMenuIndex = -1;
        if (path.endsWith(".esptxt")) {
          notepadText = "";
          loadEspTxt(path.c_str(), notepadText);
          currentMode = MODE_NOTEPAD; drawNotepad();
        } else if (path.endsWith(".espimg")) {
          uint16_t imgW, imgH; uint16_t *imgBuf = nullptr;
          if (loadEspImg(path.c_str(), imgW, imgH, imgBuf) && imgBuf) {
            fileViewerActive = true;
            fileViewerPath = path;
            lcdFillScreen(getThemeBg());
            lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
            lcdDrawText(10, 10, "VIEWER", COL_WHITE, 1);
            char vnameBuf[20]; strncpy(vnameBuf, fileList[fmi].c_str(), 19); vnameBuf[19] = 0;
            lcdDrawText(60, 10, vnameBuf, COL_YELLOW, 1);
            // EDIT IN SKETCH button
            lcdFillRect(LCD_WIDTH - 190, 3, 100, 24, COL_GREEN); lcdFillRect(LCD_WIDTH - 188, 5, 96, 20, getThemePrimary());
            lcdDrawTextCentered(LCD_WIDTH - 190, 100, 10, "EDIT SKETCH", COL_WHITE, 1);
            lcdDrawText(LCD_WIDTH - 80, 10, "TAP=BACK", COL_GRAY, 1);
            int dispW = min((int)imgW, LCD_WIDTH);
            int dispH = min((int)imgH, BAR_Y - 30);
            int ox = (LCD_WIDTH - dispW) / 2;
            int oy = 30 + (BAR_Y - 30 - dispH) / 2;
            lcdSetAddrWindow(ox, oy, ox + dispW - 1, oy + dispH - 1);
            lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
            digitalWrite(PIN_LCD_DC, HIGH);
            // Bulk write each row for speed
            static uint8_t rowBuf[LCD_WIDTH * 2];
            for (int y = 0; y < dispH; y++) {
              for (int x = 0; x < dispW; x++) {
                uint16_t px = imgBuf[y * imgW + x];
                rowBuf[x * 2] = px >> 8;
                rowBuf[x * 2 + 1] = px & 0xFF;
                // Mirror to framebuffer
                if (lcdFB) lcdFB[(oy + y) * LCD_WIDTH + (ox + x)] = px;
              }
              lcdSPI.transferBytes(rowBuf, nullptr, dispW * 2);
              if ((y & 0xF) == 0 && y > 0) { lcdSPI.endTransaction(); yield(); lcdSPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0)); digitalWrite(PIN_LCD_DC, HIGH); }
            }
            lcdSPI.endTransaction();
            free(imgBuf);
            drawControlBar();
          } else { playError(); drawFileBrowser(); }
        } else {
          playError(); fileMenuIndex = -1; drawFileBrowser();
        }
        return;
      }
      if (btnIdx == 1) { // RENAME
        uiClick();
        renameTarget = "/" + fileList[fileMenuIndex];
        // Pre-fill with current name (without extension)
        String curName = fileList[fileMenuIndex];
        int dotIdx = curName.lastIndexOf('.');
        filenameInput = dotIdx > 0 ? curName.substring(0, dotIdx) : curName;
        saveContext = 2; // rename
        saveReturnMode = MODE_FILE_BROWSER;
        fileMenuIndex = -1;
        currentMode = MODE_FILENAME_INPUT;
        drawFilenameInput();
        return;
      }
      if (btnIdx == 2) { // DELETE
        uiClick();
        String path = "/" + fileList[fileMenuIndex];
        if (SD_MMC.remove(path.c_str())) {
          playSuccess();
        } else {
          playError();
        }
        fileMenuIndex = -1;
        refreshFileList();
        drawFileBrowser();
        return;
      }
      if (btnIdx == 3) { // CANCEL
        uiClick(); fileMenuIndex = -1; drawFileBrowser(); return;
      }
      return;
    } else {
      // Tap outside menu = cancel
      uiClick(); fileMenuIndex = -1; drawFileBrowser(); return;
    }
  }
  // Scroll bar area
  if (tx >= LCD_WIDTH - 22 && fileCount > FB_MAX_ROWS) {
    int scrollH = FB_LIST_Y1 - FB_LIST_Y0;
    int thumbH = max(10, scrollH * FB_MAX_ROWS / fileCount);
    if (ty < FB_LIST_Y0 + thumbH / 2 && fileScrollOffset > 0) {
      uiClick(); fileScrollOffset = max(0, fileScrollOffset - FB_MAX_ROWS); drawFileBrowser(); return;
    } else if (ty > FB_LIST_Y1 - thumbH / 2 && fileScrollOffset < fileCount - FB_MAX_ROWS) {
      uiClick(); fileScrollOffset = min(fileCount - FB_MAX_ROWS, fileScrollOffset + FB_MAX_ROWS); drawFileBrowser(); return;
    }
    return;
  }
  // File list
  if (ty >= FB_LIST_Y0 && ty < FB_LIST_Y1) {
    int row = (ty - FB_LIST_Y0) / FB_ROW_H;
    int idx = row + fileScrollOffset;
    if (idx >= 0 && idx < fileCount) {
      uiClick();
      fileMenuIndex = idx;
      drawFileBrowser();
    }
  }
}

// ---------- DRIVER Racing Game ----------
#define DRV_ROAD_WIDTH 240
#define DRV_ROAD_X ((LCD_WIDTH - DRV_ROAD_WIDTH) / 2)
#define DRV_CAR_WIDTH 30
#define DRV_CAR_HEIGHT 40
void drawDrvBackground() {
  lcdFillScreen(0x0010);
  lcdFillRect(DRV_ROAD_X, 0, DRV_ROAD_WIDTH, LCD_HEIGHT - BAR_H, 0x4208);
  lcdFillRect(DRV_ROAD_X, 0, 4, LCD_HEIGHT - BAR_H, COL_WHITE);
  lcdFillRect(DRV_ROAD_X + DRV_ROAD_WIDTH - 4, 0, 4, LCD_HEIGHT - BAR_H, COL_WHITE);
  lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  lcdFillRect(20, 230, 80, 40, 0x2104); lcdFillRect(22, 232, 76, 36, getThemeBg()); lcdDrawTextCentered(20, 80, 242, "<", COL_WHITE, 2);
  lcdFillRect(380, 230, 80, 40, 0x2104); lcdFillRect(382, 232, 76, 36, getThemeBg()); lcdDrawTextCentered(380, 80, 242, ">", COL_WHITE, 2);
  drawControlBar();
}
void drawDrvArrows() {
  lcdFillRect(20, 230, 80, 40, (drvSteer == -1) ? COL_YELLOW : 0x2104); lcdFillRect(22, 232, 76, 36, (drvSteer == -1) ? 0x8410 : getThemeBg()); lcdDrawTextCentered(20, 80, 242, "<", COL_WHITE, 2);
  lcdFillRect(380, 230, 80, 40, (drvSteer == 1) ? COL_YELLOW : 0x2104); lcdFillRect(382, 232, 76, 36, (drvSteer == 1) ? 0x8410 : getThemeBg()); lcdDrawTextCentered(380, 80, 242, ">", COL_WHITE, 2);
}
void drawDrvNpcCar(int idx, uint16_t color) {
  int x = drvNpcs[idx].x, y = drvNpcs[idx].y;
  // NPC car body
  lcdFillRect(x - DRV_CAR_WIDTH/2, y, DRV_CAR_WIDTH, DRV_CAR_HEIGHT, color);
  lcdFillRect(x - DRV_CAR_WIDTH/2 + 5, y - 5, DRV_CAR_WIDTH - 10, 8, COL_GRAY);
  lcdFillRect(x - DRV_CAR_WIDTH/2 + 2, y + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  lcdFillRect(x + DRV_CAR_WIDTH/2 - 10, y + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  // Label "DRIVER" above the car
  lcdDrawTextCentered(x - 30, 60, y - 10, "DRIVER", COL_WHITE, 1);
}
void initDriver() {
  drvCarX = LCD_WIDTH / 2; drvScore = 0; drvSpeed = 3; drvGameOver = false;
  drvRoadOffset = 0; drvSteer = 0; drvLap = 0;
  for (int i = 0; i < DRV_MAX_NPCS; i++) {
    drvNpcs[i].x = DRV_ROAD_X + 30 + random(DRV_ROAD_WIDTH - 60);
    drvNpcs[i].y = -50 - i * 100;
    drvNpcs[i].speed = 2 + random(3);
    drvNpcs[i].active = true;
  }
  drvLastUpdate = millis();
  drawDrvBackground();
  // Draw initial road lines
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + drvRoadOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, COL_YELLOW); }
  // Draw player car
  int carY = LCD_HEIGHT - BAR_H - DRV_CAR_HEIGHT - 20;
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2, carY, DRV_CAR_WIDTH, DRV_CAR_HEIGHT, COL_RED);
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2 + 5, carY - 5, DRV_CAR_WIDTH - 10, 8, COL_YELLOW);
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2 + 2, carY + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  lcdFillRect(drvCarX + DRV_CAR_WIDTH/2 - 10, carY + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  // HUD
  char scoreStr[20]; snprintf(scoreStr, sizeof(scoreStr), "SCORE: %d", drvScore); lcdDrawText(10, 10, scoreStr, COL_WHITE, 1);
  char lapStr[20]; snprintf(lapStr, sizeof(lapStr), "LAP: %d", drvLap); lcdDrawText(LCD_WIDTH - 80, 10, lapStr, COL_WHITE, 1);
  char hiStr[20]; snprintf(hiStr, sizeof(hiStr), "HI: %d", drvHighScore); lcdDrawTextCentered(LCD_WIDTH/2 - 40, 80, 10, hiStr, COL_MINT, 1);
}
void updateDriver() {
  if (drvGameOver) return;
  unsigned long now = millis(); if (now - drvLastUpdate < 50) return; drvLastUpdate = now;
  static unsigned long drvLastEngine = 0;
  if (now - drvLastEngine > 180) { drvLastEngine = now; playEngine(drvSpeed); }
  int prevCarX = drvCarX; int prevOffset = drvRoadOffset; int prevScore = drvScore;
  // Erase old road lines
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + prevOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, 0x4208); }
  // Erase old player car
  int carY = LCD_HEIGHT - BAR_H - DRV_CAR_HEIGHT - 20;
  lcdFillRect(prevCarX - DRV_CAR_WIDTH/2, carY, DRV_CAR_WIDTH, DRV_CAR_HEIGHT, 0x4208);
  // Erase old NPC cars (constrained to road area to avoid erasing borders)
  for (int i = 0; i < DRV_MAX_NPCS; i++) {
    if (drvNpcs[i].active && drvNpcs[i].y >= 0 && drvNpcs[i].y < LCD_HEIGHT - BAR_H) {
      int eraseX = max(DRV_ROAD_X + 4, drvNpcs[i].x - DRV_CAR_WIDTH/2 - 35);
      int eraseW = min(DRV_ROAD_X + DRV_ROAD_WIDTH - 4, drvNpcs[i].x + DRV_CAR_WIDTH/2 + 35) - eraseX;
      if (eraseW > 0) lcdFillRect(eraseX, max(0, drvNpcs[i].y - 12), eraseW, DRV_CAR_HEIGHT + 15, 0x4208);
    }
  }
  // Read IMU for tilt steering
  imuReadAccel();
  int moveAmount = drvSteer * 6;
  if (drvSteer == 0 && imuAvailable) {
    if (accelX < -4000) moveAmount = -8; else if (accelX < -2000) moveAmount = -4;
    else if (accelX > 4000) moveAmount = 8; else if (accelX > 2000) moveAmount = 4;
  }
  drvCarX += moveAmount;
  int minX = DRV_ROAD_X + DRV_CAR_WIDTH/2 + 5; int maxX = DRV_ROAD_X + DRV_ROAD_WIDTH - DRV_CAR_WIDTH/2 - 5;
  if (drvCarX < minX) drvCarX = minX; if (drvCarX > maxX) drvCarX = maxX;
  drvRoadOffset = (drvRoadOffset + drvSpeed) % 40;
  // Move NPC cars (they move slower than player, so player overtakes them)
  const uint16_t npcColors[3] = {COL_BLUE, COL_GREEN, COL_YELLOW};
  for (int i = 0; i < DRV_MAX_NPCS; i++) {
    if (!drvNpcs[i].active) continue;
    drvNpcs[i].y += drvNpcs[i].speed;
    // NPC weaving
    drvNpcs[i].x += (random(3) - 1) * 2;
    if (drvNpcs[i].x < DRV_ROAD_X + 20) drvNpcs[i].x = DRV_ROAD_X + 20;
    if (drvNpcs[i].x > DRV_ROAD_X + DRV_ROAD_WIDTH - 20) drvNpcs[i].x = DRV_ROAD_X + DRV_ROAD_WIDTH - 20;
    // Collision with player
    if (drvNpcs[i].y + DRV_CAR_HEIGHT > carY && drvNpcs[i].y < carY + DRV_CAR_HEIGHT &&
        drvNpcs[i].x + DRV_CAR_WIDTH/2 > drvCarX - DRV_CAR_WIDTH/2 && drvNpcs[i].x - DRV_CAR_WIDTH/2 < drvCarX + DRV_CAR_WIDTH/2) {
      drvGameOver = true; playError();
      if (drvScore > drvHighScore) { drvHighScore = drvScore; saveHighScores(); }
      lcdFillRect(LCD_WIDTH/2 - 90, LCD_HEIGHT/2 - 40, 180, 90, COL_RED);
      lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2 - 30, "CRASH!", COL_WHITE, 2);
      char hiStr[24]; snprintf(hiStr, sizeof(hiStr), "HIGH SCORE: %d", drvHighScore);
      lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2, hiStr, COL_YELLOW, 1);
      lcdDrawTextCentered(LCD_WIDTH/2 - 90, 180, LCD_HEIGHT/2 + 20, "TAP TO RESTART", COL_WHITE, 1);
      return;
    }
    // If NPC goes off screen, respawn ahead and score
    if (drvNpcs[i].y > LCD_HEIGHT - BAR_H) {
      drvNpcs[i].y = -50 - random(100);
      drvNpcs[i].x = DRV_ROAD_X + 30 + random(DRV_ROAD_WIDTH - 60);
      drvNpcs[i].speed = 2 + random(3);
      drvScore += 10;
      if (drvScore % 50 == 0) { drvLap++; drvSpeed = min(10, drvSpeed + 1); }
    }
  }
  // Draw new road lines
  for (int y = 0; y < LCD_HEIGHT - BAR_H; y += 40) { int lineY = (y + drvRoadOffset) % (LCD_HEIGHT - BAR_H); lcdFillRect(LCD_WIDTH / 2 - 2, lineY, 4, 20, COL_YELLOW); }
  // Draw NPC cars
  for (int i = 0; i < DRV_MAX_NPCS; i++) {
    if (drvNpcs[i].active && drvNpcs[i].y >= -30 && drvNpcs[i].y < LCD_HEIGHT - BAR_H) {
      drawDrvNpcCar(i, npcColors[i]);
    }
  }
  // Draw player car
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2, carY, DRV_CAR_WIDTH, DRV_CAR_HEIGHT, COL_RED);
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2 + 5, carY - 5, DRV_CAR_WIDTH - 10, 8, COL_YELLOW);
  lcdFillRect(drvCarX - DRV_CAR_WIDTH/2 + 2, carY + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  lcdFillRect(drvCarX + DRV_CAR_WIDTH/2 - 10, carY + DRV_CAR_HEIGHT - 10, 8, 10, COL_BLACK);
  // Update HUD
  if (drvScore != prevScore) {
    lcdFillRect(10, 10, 100, 10, getThemePrimary());
    char scoreStr[20]; snprintf(scoreStr, sizeof(scoreStr), "SCORE: %d", drvScore); lcdDrawText(10, 10, scoreStr, COL_WHITE, 1);
    char lapStr[20]; snprintf(lapStr, sizeof(lapStr), "LAP: %d", drvLap); lcdDrawText(LCD_WIDTH - 80, 10, lapStr, COL_WHITE, 1);
  }
}
void handleDriverTouch(uint16_t tx, uint16_t ty) {
  if (ty >= BAR_Y) { handleControlBarTouch(tx, ty); return; }
  if (drvGameOver) { uiClick(); initDriver(); return; }
  if (tx >= 20 && tx <= 100 && ty >= 230 && ty <= 270) { if (drvSteer != -1) { drvSteer = -1; drawDrvArrows(); } return; }
  if (tx >= 380 && tx <= 460 && ty >= 230 && ty <= 270) { if (drvSteer != 1) { drvSteer = 1; drawDrvArrows(); } return; }
}

// ---------- Screen Saver Logic ----------
int ssX = 0, ssY = 0;
int ssDX = 2, ssDY = 2;
uint16_t ssColor = COL_YELLOW;
unsigned long ssStartTime = 0;
unsigned long ssLastUpdate = 0;

void resetScreenSaverFrameState();
void initScreenSaver() {
  ssX = random(LCD_WIDTH - 60);
  ssY = random(LCD_HEIGHT - 60);
  ssDX = (random(2) == 0) ? 2 : -2;
  ssDY = (random(2) == 0) ? 2 : -2;
  ssColor = COL_YELLOW;
  ssStartTime = millis();
  currentMode = MODE_SCREENSAVER;
  lcdFillScreen(COL_BLACK);
  resetScreenSaverFrameState();
}

// Previous frame's bounding box, so we only erase what we drew last time
// instead of flashing the whole screen black every few frames.
int ssPrevX = -100, ssPrevY = -100, ssPrevW = 0, ssPrevH = 0;
bool ssFirstFrame = true;

char ssLastClockStr[6] = "";

void resetScreenSaverFrameState() {
  ssPrevX = -100; ssPrevY = -100; ssPrevW = 0; ssPrevH = 0;
  ssFirstFrame = true;
  ssLastClockStr[0] = '\0';
}

void drawScreenSaverFrame() {
  // Margin around the shape's bounding box that also needs erasing (shadow/ring)
  const int PAD = 10;

  if (ssFirstFrame) {
    lcdFillScreen(COL_BLACK);
    ssFirstFrame = false;
  } else {
    // Erase only the previous frame's footprint - flicker-free
    lcdFillRect(ssPrevX - PAD, ssPrevY - PAD, ssPrevW + PAD*2, ssPrevH + PAD*2, COL_BLACK);
  }

  int w = 40, h = 40;

  if (settingSSMode == 0) { // Bouncy smile
    int cx = ssX + w/2, cy = ssY + h/2, r = w/2;
    lcdFillCircle(cx, cy, r, ssColor);                     // face
    lcdDrawCircle(cx, cy, r, COL_DKGRAY);                  // crisp outline, same radius as fill
    lcdFillCircle(cx - r/2, cy - r/3, r/6 + 2, COL_BLACK);  // left eye
    lcdFillCircle(cx + r/2, cy - r/3, r/6 + 2, COL_BLACK);  // right eye
    lcdFillCircle(cx - r/2, cy - r/3 - 1, 2, COL_WHITE);    // eye highlight
    lcdFillCircle(cx + r/2, cy - r/3 - 1, 2, COL_WHITE);    // eye highlight
    for (int i = -8; i <= 8; i++) {                        // curved smile
      int mx = cx + i;
      int my = cy + r/3 + (int)(sqrtf(64.0f - (float)(i*i)) * 0.35f);
      lcdFillRect(mx, my, 2, 2, COL_BLACK);
    }
  }
  else if (settingSSMode == 1) { // DVD-style logo card
    w = 96; h = 34;
    lcdFillRect(ssX, ssY, w, h, getThemeBg());
    lcdDrawRect(ssX, ssY, w, h, ssColor);
    lcdDrawRect(ssX + 1, ssY + 1, w - 2, h - 2, ssColor);
    lcdDrawTextCentered(ssX, w, ssY + h/2 - 4, "WAVE OS", ssColor, 1);
  }
  else if (settingSSMode == 2) { // W logo, with a subtle shadow for depth
    w = 26; h = 30;
    lcdDrawText(ssX + 2, ssY + 2, "W", COL_DKGRAY, 4);
    lcdDrawText(ssX, ssY, "W", ssColor, 4);
  }

  ssPrevX = ssX; ssPrevY = ssY; ssPrevW = w; ssPrevH = h;

  // Move
  ssX += ssDX;
  ssY += ssDY;

  // Bounce
  bool hit = false;
  if (ssX <= 0 || ssX + w >= LCD_WIDTH) {
    ssDX = -ssDX;
    ssX = constrain(ssX, 0, LCD_WIDTH - w);
    hit = true;
  }
  if (ssY <= 0 || ssY + h >= LCD_HEIGHT) {
    ssDY = -ssDY;
    ssY = constrain(ssY, 0, LCD_HEIGHT - h);
    hit = true;
  }

  if (hit) {
    // Change color on bounce
    const uint16_t colors[] = {COL_RED, COL_GREEN, COL_BLUE, COL_YELLOW, COL_MINT, COL_WHITE};
    ssColor = colors[random(6)];
  }

  // Subtle live clock, bottom-right, like a real screensaver
  char clockStr[6];
  time_t nowT = time(nullptr);
  struct tm* tI = localtime(&nowT);
  snprintf(clockStr, sizeof(clockStr), "%02d:%02d", tI->tm_hour, tI->tm_min);
  if (strcmp(clockStr, ssLastClockStr) != 0) {
    lcdFillRect(LCD_WIDTH - 60, LCD_HEIGHT - 20, 56, 14, COL_BLACK);
    lcdDrawText(LCD_WIDTH - 58, LCD_HEIGHT - 18, clockStr, COL_DKGRAY, 1);
    strcpy(ssLastClockStr, clockStr);
  }

  // The overlay HUD is drawn independently on its own 250ms timer, which is
  // much slower than the screen saver's ~30ms frame rate - without this, the
  // bouncing shape would pass under/over the HUD and never get redrawn on
  // top of it until the next 250ms tick. Reassert it every screen saver
  // frame so it always stays visible.
  if (tmOverlayEnabled) drawTmOverlay();
}

// ---------- GPU Task (Core 1 - Graphics Only) ----------
void gpuTask(void* pvParameters) {
  Serial.println("GPU Task started on Core 1");
  
  while (1) {
    gpuTaskCycles++;
    GpuTaskMessage msg;
    
    // Check for graphics commands with timeout
    if (xQueueReceive(gpuQueue, &msg, pdMS_TO_TICKS(16)) == pdTRUE) {
      // Process graphics command
      if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        switch (msg.cmd) {
          case GPU_CMD_FILL_RECT:
            lcdFillRect(msg.params[0], msg.params[1], msg.params[2], msg.params[3], msg.params[4]);
            break;
          case GPU_CMD_DRAW_TEXT:
            lcdDrawText(msg.params[0], msg.params[1], msg.text, msg.params[2], msg.params[3]);
            break;
          case GPU_CMD_FILL_SCREEN:
            lcdFillScreen(msg.params[0]);
            break;
          case GPU_CMD_DRAW_LAUNCHER:
            drawLauncher();
            break;
          case GPU_CMD_DRAW_CAMERA:
            // Camera frame drawing is handled by existing camera functions
            break;
          case GPU_CMD_DRAW_GAME:
            if (currentMode == MODE_RACING) updateRacing();
            else if (currentMode == MODE_SPACE_FIGHTERS) updateSpaceFighters();
            else if (currentMode == MODE_DRIVER) updateDriver();
            break;
          case GPU_CMD_CUSTOM_DRAW:
            if (msg.customFunc) msg.customFunc();
            break;
          default:
            break;
        }
        xSemaphoreGive(lcdMutex);
      }
    } else {
      gpuIdleTime++;
    }
    
    // Small delay to prevent Core 1 starvation
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Helper Functions for Multi-Core ----------
void processTouchInput(uint16_t tx, uint16_t ty, bool pressEdge, bool touched) {
  if (pressEdge && handleNotificationTouch(tx, ty)) return;
  if (currentMode == MODE_PIN_ENTRY) { handlePinEntryTouch(tx, ty); return; }
  if (currentMode == MODE_POWER_MENU) { handlePowerMenuTouch(tx, ty); return; }
  if (currentMode == MODE_MSGBOX) {
    if (handleMessageBoxTouch(tx, ty)) {
      uiClick();
      if (msgBoxResetPending) {
        msgBoxResetPending = false;
        settingCameraAccess = true; settingMicAccess = true; settingTouchLogging = false; settingAutoLock = false; settingClickSounds = true; settingGameSounds = true; settingThemeIndex = THEME_MINT; settingBrightness = 100; pinEnabled = false; strncpy(pinCode, "1234", 4); pinCode[4] = '\0'; saveSettings();
        showMessageBox(MSG_INFO, "SETTINGS", "ALL SETTINGS RESET", "TO DEFAULTS", MODE_SETTINGS_PRIVACY); return;
      }
      currentMode = msgBoxReturnMode;
      // Send draw command to GPU
      GpuTaskMessage msg;
      switch (currentMode) {
        case MODE_LAUNCHER: msg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
        case MODE_TICTACTOE: case MODE_TICTACTOE_AI: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawBoard; break;
        case MODE_CAMERA_DIALOG: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawCameraDialog; break;
        case MODE_SETTINGS_PRIVACY: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsPrivacy; break;
        case MODE_SETTINGS_PREFS: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsPrefs; break;
        case MODE_SETTINGS_SOUND: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsSound; break;
        case MODE_SETTINGS_TIME: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsTime; break;
        case MODE_GMAIL_SIGNIN: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawGmailSignin; break;
        case MODE_EMAIL_COMPOSE: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawEmailCompose; break;
        case MODE_EMAIL_INBOX: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawEmailInbox; break;
        case MODE_SETTINGS_SAVER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsSaver; break;
        case MODE_WIFI: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawWiFiScreen; break;
        case MODE_SETTINGS: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsMain; break;
        case MODE_START_MENU: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawStartMenu; break;
        case MODE_IP_EXPLORER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawIpExplorer; break;
        case MODE_DRIVER: initDriver(); return;
        case MODE_SPACE_FIGHTERS: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = sfGameOver ? sfShowGameOver : sfRedrawScreen; break;
        case MODE_FILE_BROWSER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawFileBrowser; break;
        case MODE_TASK_MANAGER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawTaskManager; break;
        case MODE_PIN_ENTRY: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawPinEntryScreen; break;
        default: return;
      }
      xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
    } return;
  }
  // Settings screens no longer draw the bottom control bar (it wasted 40px
  // and clipped the last card on several screens) - so they must also be
  // excluded from the generic control-bar touch gate below, or a tap on
  // content in that reclaimed space would get misread as a control-bar
  // button press instead of reaching the screen's own handler.
  bool isSettingsScreen = (currentMode == MODE_SETTINGS || currentMode == MODE_SETTINGS_SOUND ||
                            currentMode == MODE_SETTINGS_TIME || currentMode == MODE_SETTINGS_PRIVACY ||
                            currentMode == MODE_SETTINGS_PREFS || currentMode == MODE_SETTINGS_SAVER);
  if (currentMode != MODE_CAMERA_DIALOG && currentMode != MODE_WIFI_PASSWORD && currentMode != MODE_PIN_ENTRY && !isSettingsScreen) { 
    if (handleControlBarTouch(tx, ty)) return; 
  }
  if (currentMode == MODE_CAMERA_DIALOG) {
    if (handleDialogTouch(tx, ty)) {
      uiClick();
      if (cameraMode == CAM_MODE_NONE) { 
        currentMode = MODE_LAUNCHER; 
        GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      } else { 
        initCamera(); 
        if (currentMode != MODE_MSGBOX) { 
          currentMode = MODE_CAMERA; 
          GpuTaskMessage msg1 = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawControlBar};
          GpuTaskMessage msg2 = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawShutterButton};
          xQueueSend(gpuQueue, &msg1, pdMS_TO_TICKS(10));
          if (cameraMode == CAM_MODE_DIAGNOSTIC) {
            GpuTaskMessage msg3 = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawDiagnosticOverlay};
            xQueueSend(gpuQueue, &msg3, pdMS_TO_TICKS(10));
          }
          xQueueSend(gpuQueue, &msg2, pdMS_TO_TICKS(10));
        } 
      } 
    } return;
  }
  if (currentMode == MODE_WIFI_PASSWORD) { handleWifiPasswordTouch(tx, ty); return; }
  if (currentMode == MODE_CAMERA) { handleCameraRotationTouch(tx, ty); }
  if (currentMode == MODE_IP_EXPLORER) { handleIpExplorerTouch(tx, ty); return; }
  if (currentMode == MODE_IP_VIEWER) { goBack(); return; }
  if (currentMode == MODE_FILE_BROWSER) { handleFileBrowserTouch(tx, ty); return; }
  if (currentMode == MODE_TASK_MANAGER) { handleTaskManagerTouch(tx, ty); return; }
  if (currentMode == MODE_FILENAME_INPUT) { handleFilenameInputTouch(tx, ty); return; }
  if (currentMode == MODE_SD_CONFIRM) { handleSdConfirmTouch(tx, ty); return; }
  
  switch (currentMode) {
    case MODE_LAUNCHER: handleLauncherTouch(tx, ty); break;
    case MODE_TICTACTOE: case MODE_TICTACTOE_AI: handleTicTacToeTouch(tx, ty); break;
    case MODE_SKETCHPAD: handleSketchPadTouch(tx, ty); break;
    case MODE_NOTEPAD: handleNotepadTouch(tx, ty); break;
    case MODE_CAMERA: break;
    case MODE_SETTINGS: handleSettingsMainTouch(tx, ty); break;
    case MODE_SETTINGS_PRIVACY: handleSettingsPrivacyTouch(tx, ty); break;
    case MODE_SETTINGS_PREFS: handleSettingsPrefsTouch(tx, ty); break;
    case MODE_SETTINGS_SOUND: handleSettingsSoundTouch(tx, ty); break;
    case MODE_SETTINGS_TIME: handleSettingsTimeTouch(tx, ty); break;
    case MODE_GMAIL_SIGNIN: handleGmailSigninTouch(tx, ty); break;
    case MODE_EMAIL_INBOX: handleEmailInboxTouch(tx, ty); break;
    case MODE_EMAIL_COMPOSE: handleEmailComposeTouch(tx, ty); break;
    case MODE_SETTINGS_SAVER: handleSettingsSaverTouch(tx, ty); break;
    case MODE_WIFI: handleWifiTouch(tx, ty); break;
    case MODE_CHECKERS: case MODE_CHECKERS_AI: handleCheckersTouch(tx, ty); break;
    case MODE_MINESWEEPER: handleMinesweeperTouch(tx, ty); break;
    case MODE_RACING: handleRacingTouch(tx, ty); break;
    case MODE_RACING_DIFFICULTY: handleRacingDifficultyTouch(tx, ty); break;
    case MODE_CLOCK: handleClockTouch(tx, ty); break;
    case MODE_START_MENU: handleStartMenuTouch(tx, ty); break;
    case MODE_SPACE_FIGHTERS: handleSpaceFightersTouch(tx, ty); break;
    case MODE_SF_DIFFICULTY: handleSFDifficultyTouch(tx, ty); break;
    case MODE_DRIVER: handleDriverTouch(tx, ty); break;
    case MODE_TASK_MANAGER: handleTaskManagerTouch(tx, ty); break;
    case MODE_WSHOP: handleWShopTouch(tx, ty); break;
    default: break;
  }
}

void processHoldLogic(uint16_t tx, uint16_t ty, bool touched) {
  // Start Menu hold-to-remove logic
  if (currentMode == MODE_START_MENU && appHoldIdx >= 0) {
    if (touched && !appHoldFired) {
      int tabH = 30; int gridCols = 4; int gridRows = 4;
      int cellW = LCD_WIDTH / gridCols; int cellH = (LCD_HEIGHT - BAR_H - tabH) / gridRows;
      int startY = tabH;
      int idx = 0, foundR = -1, foundC = -1;
      for (int r = 0; r < gridRows && foundR < 0; r++) {
        for (int c = 0; c < gridCols; c++) {
          while (idx < startAppCount) { if (startMenuFilterPass(idx)) break; idx++; }
          if (idx >= startAppCount) break;
          if (idx == appHoldIdx) { foundR = r; foundC = c; break; }
          idx++;
        }
      }
      if (foundR < 0) { appHoldIdx = -1; appHoldStart = 0; appHoldFired = false; }
      else {
        int x = foundC * cellW, y = startY + foundR * cellH;
        if (tx < x || tx >= x + cellW || ty < y || ty >= y + cellH) {
          appHoldIdx = -1; appHoldStart = 0; appHoldFired = false;
        } else {
          unsigned long held = millis() - appHoldStart;
          if (held >= 700) {
            appHoldFired = true;
            playRemoveApp();
            startAppRemoved[appHoldIdx] = true;
            saveSettings();
            GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawStartMenu};
            xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
          } else {
            int pct = (int)(held * 100 / 700);
            int barW = (cellW - 8) * pct / 100;
            GpuTaskMessage msg = {GPU_CMD_FILL_RECT, {x + 4, y + cellH - 6, barW, 3, COL_RED}};
            xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
          }
        }
      }
    } else if (!touched) {
      if (!appHoldFired) launchStartMenuApp(appHoldIdx);
      appHoldIdx = -1; appHoldStart = 0; appHoldFired = false;
    }
  }
  
  // Launcher hold-to-remove logic
  if (currentMode == MODE_LAUNCHER && launcherHoldIdx >= 0) {
    int idx = launcherHoldIdx;
    int appId = 0, foundR = -1, foundC = -1;
    for (int r = 0; r < GRID_ROWS && foundR < 0; r++) {
      for (int c = 0; c < GRID_COLS; c++) {
        while (appId < MAX_APPS && !launcherAppVisible(appId)) appId++;
        if (appId >= MAX_APPS) break;
        if (appId == idx) { foundR = r; foundC = c; break; }
        appId++;
      }
    }
    if (foundR < 0) { launcherHoldIdx = -1; launcherHoldStart = 0; launcherHoldFired = false; }
    else {
      int x = foundC * CELL_W, y = foundR * CELL_H;
      if (touched && !launcherHoldFired) {
        if (tx < x || tx >= x + CELL_W || ty < y || ty >= y + CELL_H) {
          launcherHoldIdx = -1; launcherHoldStart = 0; launcherHoldFired = false;
        } else {
          unsigned long held = millis() - launcherHoldStart;
          if (held >= 700) {
            launcherHoldFired = true;
            int mappedIdx = launcherToStartIdx[idx];
            if (mappedIdx >= 0) {
              playRemoveApp();
              startAppRemoved[mappedIdx] = true;
              saveSettings();
              GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
              xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
            } else {
              playError();
            }
          } else {
            int pct = (int)(held * 100 / 700);
            int barW = (CELL_W - 8) * pct / 100;
            GpuTaskMessage msg = {GPU_CMD_FILL_RECT, {x + 4, y + CELL_H - 6, barW, 3, COL_RED}};
            xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
          }
        }
      } else if (!touched) {
        if (!launcherHoldFired) launchLauncherApp(idx);
        launcherHoldIdx = -1; launcherHoldStart = 0; launcherHoldFired = false;
      }
    }
  }
}

void updateScreenSaverLogic() {
  if (millis() - ssLastUpdate > 30) {
    ssLastUpdate = millis();
    drawScreenSaverFrame();
  }
}

// ---------- Missing Helper Functions ----------
void processCameraFrame() {
  // Camera frame processing logic (non-graphics part)
  // This function handles the actual camera data processing
  // Graphics rendering is handled by the GPU task
  if (cameraMode == CAM_MODE_LIVE) {
    // Send camera frame draw command to GPU
    GpuTaskMessage msg = {GPU_CMD_DRAW_CAMERA};
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  }
}

// ---------- CPU Task (Core 0 - All Non-Graphics Operations) ----------
void cpuTask(void* pvParameters) {
  Serial.println("CPU Task started on Core 0");
  
  while (1) {
    cpuTaskCycles++;
    unsigned long loopStart = millis();
    
    // Handle touch input
    uint16_t tx, ty;
    bool touched = ft6336GetTouch(tx, ty);
    static bool wasTouched = false;
    bool pressEdge = touched && !wasTouched;
    wasTouched = touched;
    if (touched) { if (tx >= LCD_WIDTH) tx = LCD_WIDTH-1; if (ty >= LCD_HEIGHT) ty = LCD_HEIGHT-1; }

    // ---- Draggable overlay HUD ----
    // Handled here, before any mode-specific dispatch, so the HUD can be
    // grabbed and moved from ANY screen (task manager, launcher, a game,
    // even the screen saver) without that screen's own touch handler
    // stealing the press first.
    bool overlayConsumedTouch = false;
    if (tmOverlayEnabled) {
      bool insideOverlay = touched && tx >= tmOverlayX && tx < tmOverlayX + TM_OVERLAY_W &&
                            ty >= tmOverlayY && ty < tmOverlayY + TM_OVERLAY_H;
      if (pressEdge && insideOverlay) {
        tmOverlayDragging = true;
        tmOverlayDragOffX = tx - tmOverlayX;
        tmOverlayDragOffY = ty - tmOverlayY;
        overlayConsumedTouch = true;
      } else if (tmOverlayDragging && touched) {
        int newX = (int)tx - tmOverlayDragOffX;
        int newY = (int)ty - tmOverlayDragOffY;
        if (newX < 0) newX = 0; if (newY < 0) newY = 0;
        if (newX > LCD_WIDTH - TM_OVERLAY_W) newX = LCD_WIDTH - TM_OVERLAY_W;
        if (newY > LCD_HEIGHT - TM_OVERLAY_H) newY = LCD_HEIGHT - TM_OVERLAY_H;
        if ((newX != tmOverlayX || newY != tmOverlayY) &&
            millis() - tmOverlayLastDragRedraw >= TM_OVERLAY_DRAG_REDRAW_MS) {
          tmOverlayLastDragRedraw = millis();
          tmOverlayX = newX; tmOverlayY = newY;
          redrawUnderlyingModeThenOverlay(); // repaints the spot just vacated, then the HUD at its new spot
        }
        overlayConsumedTouch = true;
      } else if (tmOverlayDragging && !touched) {
        tmOverlayDragging = false;
        redrawUnderlyingModeThenOverlay(); // final settle so the last vacated spot is clean
        overlayConsumedTouch = true;
      }
    }

    if (overlayConsumedTouch) pressEdge = false; // don't let the underlying screen also react to this touch

    // Screenshot handling
    bool bootPressed = (digitalRead(BOOT_PIN) == LOW);
    if (bootPressed) {
      if (bootPressStart == 0) bootPressStart = millis();
      else if (!bootHeld && (millis() - bootPressStart >= 2000)) {
        bootHeld = true;
        takeScreenshot();
      }
    } else {
      bootPressStart = 0;
      bootHeld = false;
    }
    
    // Mode-specific non-graphics logic
    static bool wasNotificationActive = false;
    updateNotification();
    if (notificationActive != wasNotificationActive) {
      wasNotificationActive = notificationActive;
      GpuTaskMessage nmsg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawNotification};
      xQueueSend(gpuQueue, &nmsg, pdMS_TO_TICKS(10));
      if (!notificationActive) {
        // Notification just expired - redraw whatever's underneath the strip
        GpuTaskMessage rmsg = {GPU_CMD_NONE};
        switch (currentMode) {
          case MODE_LAUNCHER: rmsg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
          case MODE_IP_EXPLORER: rmsg.cmd = GPU_CMD_CUSTOM_DRAW; rmsg.customFunc = drawIpExplorer; break;
          case MODE_EMAIL_INBOX: rmsg.cmd = GPU_CMD_CUSTOM_DRAW; rmsg.customFunc = drawEmailInbox; break;
          case MODE_TASK_MANAGER: rmsg.cmd = GPU_CMD_CUSTOM_DRAW; rmsg.customFunc = drawTaskManager; break;
          case MODE_FILE_BROWSER: rmsg.cmd = GPU_CMD_CUSTOM_DRAW; rmsg.customFunc = drawFileBrowser; break;
          default: break;
        }
        if (rmsg.cmd != GPU_CMD_NONE) xQueueSend(gpuQueue, &rmsg, pdMS_TO_TICKS(10));
      }
    }

    // ---- Task Manager: 250ms live data refresh (partial/changed-pixel redraw) ----
    if (millis() - tmLastRefresh >= TM_REFRESH_INTERVAL_MS) {
      tmLastRefresh = millis();
      if (currentMode == MODE_TASK_MANAGER) {
        GpuTaskMessage tmsg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, tmRefreshTick};
        xQueueSend(gpuQueue, &tmsg, pdMS_TO_TICKS(10));
      } else if (tmOverlayEnabled) {
        // App isn't open but the floating overlay HUD stays on - only its
        // small fixed region gets touched, never a full-screen redraw.
        GpuTaskMessage omsg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTmOverlay};
        xQueueSend(gpuQueue, &omsg, pdMS_TO_TICKS(10));
      }
    }

    // ---- IP Explorer: non-blocking background scan step ----
    // Runs regardless of currentMode so the scan keeps going after the user
    // backs out of IP Explorer into another app.
    if (ipScanning) {
      if (!wifiConnected) {
        ipScanCancelled = true; // WiFi dropped - abort and save what we found
        ipScanFinish(true);
      } else if (pressEdge) {
        // A fresh touch just landed this pass (e.g. EXIT / STOP SCAN) - skip
        // the probe this iteration so processTouchInput() below handles it
        // immediately instead of waiting behind a blocking network call.
      } else if (millis() - ipScanLastStepTime >= IP_SCAN_STEP_INTERVAL_MS) {
        ipScanLastStepTime = millis();
        ipScanStep();
      }
    }

    if (currentMode == MODE_SPLASH) { 
      updateSplash(); 
    } else if (currentMode == MODE_SCREENSAVER) {
      if (touched) {
        currentMode = MODE_LAUNCHER;
        lastActivityTime = millis();
        // Send graphics command to GPU
        GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      } else {
        // Screensaver animation logic
        updateScreenSaverLogic();
      }
    } else if (currentMode == MODE_GHOSTING) {
      // Ghosting app logic
      if (ssAutoTriggered && touched) {
        ssAutoTriggered = false;
        currentMode = MODE_LAUNCHER;
        lastActivityTime = millis();
        GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (millis() - ghostingLastToggle > 1000) {
        ghostingState = !ghostingState;
        GpuTaskMessage msg = {GPU_CMD_FILL_SCREEN, {ghostingState ? COL_WHITE : COL_BLACK}};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
        ghostingLastToggle = millis();
      }
      if (millis() - ghostingStartTime >= 30 * 60 * 1000UL) {
        ssAutoTriggered = false;
        currentMode = MODE_LAUNCHER;
        GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
    } else {
      // Handle touch events for all other modes
      if (touched) lastActivityTime = millis();
      
      // Screen saver trigger check
      if (settingScreensaverEnabled && currentMode != MODE_SCREENSAVER && 
          currentMode != MODE_GHOSTING && currentMode != MODE_GHOSTING_WARNING) {
        if (millis() - lastActivityTime > (unsigned long)settingSSTimeout * 60 * 1000UL) {
          initScreenSaver();
        }
      }
      
      // Process touch input
      bool actionable = (currentMode == MODE_SKETCHPAD) ? 
        (pressEdge || (ty >= SKETCH_TOOLBAR_H && ty < BAR_Y && touched)) : pressEdge;
      
      if (actionable) {
        processTouchInput(tx, ty, pressEdge, touched);
      }
      
      // Game updates (send to GPU for rendering)
      if (currentMode == MODE_RACING && !touched && steerInput != 0) { 
        steerInput = 0; 
        // Send steering update to GPU
        GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawSteeringArrows};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (currentMode == MODE_RACING) {
        GpuTaskMessage msg = {GPU_CMD_DRAW_GAME};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (currentMode == MODE_SPACE_FIGHTERS && !touched && sfSteer != 0) { 
        sfSteer = 0; 
        GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawSFArrows};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (currentMode == MODE_SPACE_FIGHTERS) {
        GpuTaskMessage msg = {GPU_CMD_DRAW_GAME};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (currentMode == MODE_DRIVER && !touched && drvSteer != 0) { 
        drvSteer = 0; 
        GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawDrvArrows};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      if (currentMode == MODE_DRIVER) {
        GpuTaskMessage msg = {GPU_CMD_DRAW_GAME};
        xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      }
      
      // Hold-to-remove logic for launcher and start menu
      processHoldLogic(tx, ty, touched);
    }
    
    // Camera frame processing (non-graphics part)
    if (currentMode == MODE_CAMERA && cameraMode == CAM_MODE_LIVE) {
      processCameraFrame();
    }
    
    // Update CPU statistics
    unsigned long loopTime = millis() - loopStart;
    if (loopTime < 16) {
      cpuIdleTime += (16 - loopTime);
    }
    
    // Small delay to prevent Core 0 starvation
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Task Manager App (Core Usage Monitor) ----------
void drawTaskManager() {
  // NOTE: no lcdMutex take/give here. This function is only ever invoked as a
  // GPU_CMD_CUSTOM_DRAW callback from inside gpuTask(), which already holds
  // lcdMutex while calling msg.customFunc(). Taking it again here caused an
  // immediate self-deadlock (FreeRTOS mutexes are not recursive) that timed
  // out after 100ms and skipped the whole draw - i.e. Task Manager never
  // actually rendered anything.
  // Calculate CPU and GPU usage percentages
  unsigned long totalCpuTime = cpuTaskCycles + cpuIdleTime;
  unsigned long totalGpuTime = gpuTaskCycles + gpuIdleTime;
  int cpuUsage = (totalCpuTime > 0) ? (int)((cpuTaskCycles * 100) / totalCpuTime) : 0;
  int gpuUsage = (totalGpuTime > 0) ? (int)((gpuTaskCycles * 100) / totalGpuTime) : 0;

  // Update history for graphs
  cpuHistory[historyIndex] = cpuUsage;
  gpuHistory[historyIndex] = gpuUsage;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  // Reset counters periodically
  if (millis() - lastCpuStatsUpdate > 1000) {
    cpuTaskCycles = 0;
    cpuIdleTime = 0;
    gpuTaskCycles = 0;
    gpuIdleTime = 0;
    lastCpuStatsUpdate = millis();
  }

  // Draw main screen background
  lcdFillScreen(getThemeBg());

  // Title bar
  lcdFillRect(0, 0, LCD_WIDTH, 30, getThemePrimary());
  lcdDrawText(10, 8, "TASK MANAGER", COL_WHITE, 2);

  // OVERLAY toggle - shows a small floating CPU/GPU HUD on top of any app
  lcdFillRect(LCD_WIDTH - 76, 4, 72, 22, tmOverlayEnabled ? COL_GREEN : getThemeSecondary());
  lcdDrawRect(LCD_WIDTH - 76, 4, 72, 22, COL_WHITE);
  lcdDrawTextCentered(LCD_WIDTH - 76, 72, 10, tmOverlayEnabled ? "OVR:ON" : "OVR:OFF", COL_WHITE, 1);

  // Tab bar
  int tabW = LCD_WIDTH / TM_TAB_COUNT;
  for (int i = 0; i < TM_TAB_COUNT; i++) {
    int tabX = i * tabW;
    if (i == taskManagerTab) {
      lcdFillRect(tabX, 30, tabW, 25, getThemeSecondary());
      lcdDrawRect(tabX, 30, tabW, 25, COL_WHITE);
    } else {
      lcdFillRect(tabX, 30, tabW, 25, getThemePrimary());
    }
    lcdDrawTextCentered(tabX, tabW, 37, tmTabNames[i], COL_WHITE, 1);
  }

  // Full draw just happened - force the next 250ms tick to also draw fresh
  // rather than assume the old "last drawn" pixel state is still on screen.
  tmLastCpuUsage = -1; tmLastGpuUsage = -1;
  tmLastCpuBarW = -1; tmLastGpuBarW = -1;
  tmLastGraphCol = -1;

  // Draw selected tab content
  switch (taskManagerTab) {
    case 0: drawTaskManagerOverview(); break;
    case 1: drawTaskManagerPerformance(); break;
    case 2: drawTaskManagerTasks(); break;
  }
}

// ---------- Task Manager: 250ms live refresh (changed-pixels-only) ----------
// Called every 250ms while Task Manager is open. Unlike drawTaskManager(),
// this NEVER repaints the title bar, tab bar, or static labels - it only
// touches the specific pixels whose value actually changed since the last
// tick: the bar-fill deltas, the percentage digits, and (on the Performance
// tab) the single newest graph column.
void tmRefreshTick() {
  unsigned long totalCpuTime = cpuTaskCycles + cpuIdleTime;
  unsigned long totalGpuTime = gpuTaskCycles + gpuIdleTime;
  int cpuUsage = (totalCpuTime > 0) ? (int)((cpuTaskCycles * 100) / totalCpuTime) : 0;
  int gpuUsage = (totalGpuTime > 0) ? (int)((gpuTaskCycles * 100) / totalGpuTime) : 0;

  cpuHistory[historyIndex] = cpuUsage;
  gpuHistory[historyIndex] = gpuUsage;
  int newCol = historyIndex;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  if (millis() - lastCpuStatsUpdate > 1000) {
    cpuTaskCycles = 0; cpuIdleTime = 0; gpuTaskCycles = 0; gpuIdleTime = 0;
    lastCpuStatsUpdate = millis();
  }

  bool changed = (cpuUsage != tmLastCpuUsage) || (gpuUsage != tmLastGpuUsage);

  if (taskManagerTab == 0 && changed) {
    tmPartialUpdateOverviewBars(cpuUsage, gpuUsage);
  } else if (taskManagerTab == 1) {
    tmPartialUpdateGraphColumn(newCol, cpuUsage, gpuUsage);
  }
  // TASKS tab (2) has nothing time-varying worth repainting every tick.

  tmLastCpuUsage = cpuUsage;
  tmLastGpuUsage = gpuUsage;

  if (tmOverlayEnabled) drawTmOverlay();
}

// ---------- Floating overlay HUD ----------
// Small fixed-size box drawn on top of whatever is currently on screen.
// Always repaints (no "skip if unchanged" shortcut) - fast-moving content
// underneath it (e.g. the screen saver's bouncing shape) can overdraw part
// of the HUD between ticks, so it needs to reassert itself every call
// rather than only when the CPU/GPU numbers themselves changed.
void drawTmOverlay() {
  unsigned long totalCpuTime = cpuTaskCycles + cpuIdleTime;
  unsigned long totalGpuTime = gpuTaskCycles + gpuIdleTime;
  int cpuUsage = (totalCpuTime > 0) ? (int)((cpuTaskCycles * 100) / totalCpuTime) : 0;
  int gpuUsage = (totalGpuTime > 0) ? (int)((gpuTaskCycles * 100) / totalGpuTime) : 0;

  lcdFillRect(tmOverlayX, tmOverlayY, TM_OVERLAY_W, TM_OVERLAY_H, COL_BLACK);
  lcdDrawRect(tmOverlayX, tmOverlayY, TM_OVERLAY_W, TM_OVERLAY_H, COL_MINT);
  char line[24];
  snprintf(line, sizeof(line), "C%d%% G%d%%", cpuUsage, gpuUsage);
  lcdDrawTextCentered(tmOverlayX, TM_OVERLAY_W, tmOverlayY + 8, line, COL_WHITE, 1);

  tmOverlayLastCpu = cpuUsage;
  tmOverlayLastGpu = gpuUsage;
  tmOverlayLastX = tmOverlayX;
  tmOverlayLastY = tmOverlayY;
}

// Redraws whatever screen is currently underneath the overlay (via the
// normal per-mode GPU draw command) then re-paints the overlay on top at
// its current position. Used when the overlay moves, so the spot it just
// vacated gets its real content back instead of a leftover black box.
void redrawUnderlyingModeThenOverlay() {
  GpuTaskMessage msg = {GPU_CMD_NONE};
  switch (currentMode) {
    case MODE_LAUNCHER: msg.cmd = GPU_CMD_DRAW_LAUNCHER; break;
    case MODE_IP_EXPLORER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawIpExplorer; break;
    case MODE_EMAIL_INBOX: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawEmailInbox; break;
    case MODE_TASK_MANAGER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawTaskManager; break;
    case MODE_FILE_BROWSER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawFileBrowser; break;
    case MODE_SCREENSAVER: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = resetScreenSaverFrameState; break;
    case MODE_SETTINGS: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawSettingsMain; break;
    case MODE_WIFI: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawWiFiScreen; break;
    case MODE_NOTEPAD: msg.cmd = GPU_CMD_CUSTOM_DRAW; msg.customFunc = drawNotepad; break;
    default: break;
  }
  if (msg.cmd != GPU_CMD_NONE) xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
  GpuTaskMessage omsg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTmOverlay};
  xQueueSend(gpuQueue, &omsg, pdMS_TO_TICKS(10));
}

void eraseTmOverlay() {
  // Used when overlay is turned off mid-session; caller is responsible for
  // redrawing whatever mode is currently active afterward.
  tmOverlayLastCpu = -1; tmOverlayLastGpu = -1;
}

void drawTaskManagerOverview() {
  // Calculate current usage
  unsigned long totalCpuTime = cpuTaskCycles + cpuIdleTime;
  unsigned long totalGpuTime = gpuTaskCycles + gpuIdleTime;
  int cpuUsage = (totalCpuTime > 0) ? (int)((cpuTaskCycles * 100) / totalCpuTime) : 0;
  int gpuUsage = (totalGpuTime > 0) ? (int)((gpuTaskCycles * 100) / totalGpuTime) : 0;
  
  int y = 65;
  
  // CPU Core (Core 0) section
  lcdDrawText(20, y, "CORE 0 (CPU)", COL_WHITE, 2);
  lcdDrawText(20, y + 30, "Functions: Input, WiFi, Camera, FS", COL_GRAY, 1);
  
  // CPU usage bar
  lcdFillRect(20, y + 60, 300, 20, COL_DKGRAY);
  int cpuBarWidth = (cpuUsage * 300) / 100;
  lcdFillRect(20, y + 60, cpuBarWidth, 20, COL_GREEN);
  char cpuStr[32];
  snprintf(cpuStr, sizeof(cpuStr), "Usage: %d%%", cpuUsage);
  lcdDrawText(330, y + 65, cpuStr, COL_WHITE, 1);
  tmLastCpuBarW = cpuBarWidth; // remember what's actually on screen now

  // GPU Core (Core 1) section
  lcdDrawText(20, y + 110, "CORE 1 (GPU)", COL_WHITE, 2);
  lcdDrawText(20, y + 140, "Functions: Graphics, Display, UI", COL_GRAY, 1);
  
  // GPU usage bar
  lcdFillRect(20, y + 170, 300, 20, COL_DKGRAY);
  int gpuBarWidth = (gpuUsage * 300) / 100;
  lcdFillRect(20, y + 170, gpuBarWidth, 20, COL_BLUE);
  char gpuStr[32];
  snprintf(gpuStr, sizeof(gpuStr), "Usage: %d%%", gpuUsage);
  lcdDrawText(330, y + 175, gpuStr, COL_WHITE, 1);
  tmLastGpuBarW = gpuBarWidth; // remember what's actually on screen now

  // System info
  lcdDrawText(20, y + 220, "Total PSRAM:", COL_GRAY, 1);
  char psramStr[32];
  snprintf(psramStr, sizeof(psramStr), "%d KB", ESP.getPsramSize() / 1024);
  lcdDrawText(120, y + 220, psramStr, COL_WHITE, 1);
  
  lcdDrawText(220, y + 220, "Free PSRAM:", COL_GRAY, 1);
  char freePsramStr[32];
  snprintf(freePsramStr, sizeof(freePsramStr), "%d KB", ESP.getFreePsram() / 1024);
  lcdDrawText(320, y + 220, freePsramStr, COL_WHITE, 1);
  
  // Current mode display
  lcdDrawText(20, y + 250, "Current Mode:", COL_GRAY, 1);
  const char* modeStr = "UNKNOWN";
  switch (currentMode) {
    case MODE_LAUNCHER: modeStr = "LAUNCHER"; break;
    case MODE_CAMERA: modeStr = "CAMERA"; break;
    case MODE_RACING: modeStr = "RACING"; break;
    case MODE_SPACE_FIGHTERS: modeStr = "SPACE FIGHTERS"; break;
    case MODE_DRIVER: modeStr = "DRIVER"; break;
    case MODE_SETTINGS: modeStr = "SETTINGS"; break;
    case MODE_WIFI: modeStr = "WIFI"; break;
    case MODE_TASK_MANAGER: modeStr = "TASK MANAGER"; break;
    default: modeStr = "OTHER"; break;
  }
  lcdDrawText(120, y + 250, modeStr, COL_WHITE, 1);
}

// Only repaints the bar-width delta and the percentage digits - the labels,
// borders, and everything else on the Overview tab are left untouched.
void tmPartialUpdateOverviewBars(int cpuUsage, int gpuUsage) {
  int y = 65;
  int barX = 20, barW = 300, barH = 20;
  int cpuBarY = y + 60, gpuBarY = y + 170;

  int newCpuW = (cpuUsage * barW) / 100;
  if (tmLastCpuBarW < 0) tmLastCpuBarW = newCpuW;
  if (newCpuW != tmLastCpuBarW) {
    if (newCpuW > tmLastCpuBarW) lcdFillRect(barX + tmLastCpuBarW, cpuBarY, newCpuW - tmLastCpuBarW, barH, COL_GREEN);
    else lcdFillRect(barX + newCpuW, cpuBarY, tmLastCpuBarW - newCpuW, barH, COL_DKGRAY);
    tmLastCpuBarW = newCpuW;
  }
  lcdFillRect(330, y + 65, 100, 10, getThemeBg());
  char cpuStr[32]; snprintf(cpuStr, sizeof(cpuStr), "Usage: %d%%", cpuUsage);
  lcdDrawText(330, y + 65, cpuStr, COL_WHITE, 1);

  int newGpuW = (gpuUsage * barW) / 100;
  if (tmLastGpuBarW < 0) tmLastGpuBarW = newGpuW;
  if (newGpuW != tmLastGpuBarW) {
    if (newGpuW > tmLastGpuBarW) lcdFillRect(barX + tmLastGpuBarW, gpuBarY, newGpuW - tmLastGpuBarW, barH, COL_BLUE);
    else lcdFillRect(barX + newGpuW, gpuBarY, tmLastGpuBarW - newGpuW, barH, COL_DKGRAY);
    tmLastGpuBarW = newGpuW;
  }
  lcdFillRect(330, y + 175, 100, 10, getThemeBg());
  char gpuStr[32]; snprintf(gpuStr, sizeof(gpuStr), "Usage: %d%%", gpuUsage);
  lcdDrawText(330, y + 175, gpuStr, COL_WHITE, 1);
}

// Scrolls the graph interior one column left inside the PSRAM framebuffer
// mirror and blits ONLY that changed rect back to the LCD, then draws just
// the newest line segment - instead of recomputing and redrawing all 59
// segments of both graphs every 250ms like a full redraw would.
void tmPartialUpdateGraphColumn(int col, int cpuUsage, int gpuUsage) {
  if (!lcdFB) return; // no PSRAM mirror available - skip partial scroll, full redraw on tab entry still works
  int graphX = 20, graphY = 80, graphW = 440, graphH = 100;
  int gpuGraphY = graphY + graphH + 30;
  int colW = graphW / HISTORY_SIZE; if (colW < 1) colW = 1;
  int innerW = graphW - 2 - colW;
  if (innerW <= 0) return;

  for (int row = 0; row < graphH - 2; row++) {
    uint16_t *r = lcdFB + (graphY + 1 + row) * LCD_WIDTH + (graphX + 1);
    memmove(r, r + colW, (size_t)innerW * 2);
    for (int i = 0; i < colW; i++) r[innerW + i] = COL_BLACK;
  }
  lcdBlitRectFromFB(graphX + 1, graphY + 1, graphW - 2, graphH - 2);

  for (int row = 0; row < graphH - 2; row++) {
    uint16_t *r = lcdFB + (gpuGraphY + 1 + row) * LCD_WIDTH + (graphX + 1);
    memmove(r, r + colW, (size_t)innerW * 2);
    for (int i = 0; i < colW; i++) r[innerW + i] = COL_BLACK;
  }
  lcdBlitRectFromFB(graphX + 1, gpuGraphY + 1, graphW - 2, graphH - 2);

  int prevIdx = (col - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  int xPrev = graphX + graphW - 2 - colW, xNew = graphX + graphW - 2;

  int yPrevCpu = graphY + graphH - (cpuHistory[prevIdx] * graphH) / 100;
  int yNewCpu  = graphY + graphH - (cpuUsage * graphH) / 100;
  lcdDrawLine(xPrev, yPrevCpu, xNew, yNewCpu, COL_GREEN);

  int yPrevGpu = gpuGraphY + graphH - (gpuHistory[prevIdx] * graphH) / 100;
  int yNewGpu  = gpuGraphY + graphH - (gpuUsage * graphH) / 100;
  lcdDrawLine(xPrev, yPrevGpu, xNew, yNewGpu, COL_BLUE);
}

void drawTaskManagerPerformance() {
  int graphX = 20;
  int graphY = 80;
  int graphW = 440;
  int graphH = 100;
  
  // CPU Graph
  lcdDrawText(20, 65, "CPU HISTORY (Core 0)", COL_GREEN, 1);
  lcdFillRect(graphX, graphY, graphW, graphH, COL_BLACK);
  lcdDrawRect(graphX, graphY, graphW, graphH, COL_WHITE);
  
  // Draw CPU graph line
  for (int i = 0; i < HISTORY_SIZE - 1; i++) {
    int idx1 = (historyIndex + i) % HISTORY_SIZE;
    int idx2 = (historyIndex + i + 1) % HISTORY_SIZE;
    int x1 = graphX + (i * graphW) / HISTORY_SIZE;
    int x2 = graphX + ((i + 1) * graphW) / HISTORY_SIZE;
    int y1 = graphY + graphH - (cpuHistory[idx1] * graphH) / 100;
    int y2 = graphY + graphH - (cpuHistory[idx2] * graphH) / 100;
    lcdDrawLine(x1, y1, x2, y2, COL_GREEN);
  }
  
  // GPU Graph
  int gpuGraphY = graphY + graphH + 30;
  lcdDrawText(20, gpuGraphY - 15, "GPU HISTORY (Core 1)", COL_BLUE, 1);
  lcdFillRect(graphX, gpuGraphY, graphW, graphH, COL_BLACK);
  lcdDrawRect(graphX, gpuGraphY, graphW, graphH, COL_WHITE);
  
  // Draw GPU graph line
  for (int i = 0; i < HISTORY_SIZE - 1; i++) {
    int idx1 = (historyIndex + i) % HISTORY_SIZE;
    int idx2 = (historyIndex + i + 1) % HISTORY_SIZE;
    int x1 = graphX + (i * graphW) / HISTORY_SIZE;
    int x2 = graphX + ((i + 1) * graphW) / HISTORY_SIZE;
    int y1 = gpuGraphY + graphH - (gpuHistory[idx1] * graphH) / 100;
    int y2 = gpuGraphY + graphH - (gpuHistory[idx2] * graphH) / 100;
    lcdDrawLine(x1, y1, x2, y2, COL_BLUE);
  }
  
  // Legend
  lcdFillRect(20, 300, 20, 10, COL_GREEN);
  lcdDrawText(45, 298, "CPU", COL_WHITE, 1);
  lcdFillRect(100, 300, 20, 10, COL_BLUE);
  lcdDrawText(125, 298, "GPU", COL_WHITE, 1);
}

void drawTaskManagerTasks() {
  int y = 70;
  
  // CPU Task (Core 0)
  lcdFillRect(20, y, 440, 60, getThemeSecondary());
  lcdDrawRect(20, y, 440, 60, COL_WHITE);
  lcdDrawText(30, y + 10, "CPU TASK (Core 0)", COL_GREEN, 2);
  lcdDrawText(30, y + 35, "Status: RUNNING | Priority: 1 | Stack: 8KB", COL_GRAY, 1);
  lcdDrawText(300, y + 35, "Core: 0", COL_WHITE, 1);
  
  // GPU Task (Core 1)
  y += 70;
  lcdFillRect(20, y, 440, 60, getThemeSecondary());
  lcdDrawRect(20, y, 440, 60, COL_WHITE);
  lcdDrawText(30, y + 10, "GPU TASK (Core 1)", COL_BLUE, 2);
  lcdDrawText(30, y + 35, "Status: RUNNING | Priority: 2 | Stack: 8KB", COL_GRAY, 1);
  lcdDrawText(300, y + 35, "Core: 1", COL_WHITE, 1);
  
  // Main Loop Task
  y += 70;
  lcdFillRect(20, y, 440, 60, getThemeSecondary());
  lcdDrawRect(20, y, 440, 60, COL_WHITE);
  lcdDrawText(30, y + 10, "MAIN LOOP", COL_YELLOW, 2);
  lcdDrawText(30, y + 35, "Status: IDLE | Framework Compatibility", COL_GRAY, 1);
  lcdDrawText(300, y + 35, "Core: 0", COL_WHITE, 1);
  
  // Total Tasks Info
  y += 70;
  lcdDrawText(20, y, "Total Running Tasks: 3", COL_WHITE, 1);
  lcdDrawText(200, y, "Cores Active: 2/2", COL_WHITE, 1);
}

void handleTaskManagerTouch(uint16_t tx, uint16_t ty) {
  // OVERLAY toggle button (top-right of title bar)
  if (tx >= LCD_WIDTH - 76 && tx < LCD_WIDTH - 4 && ty >= 4 && ty < 26) {
    uiClick();
    tmOverlayEnabled = !tmOverlayEnabled;
    if (!tmOverlayEnabled) eraseTmOverlay();
    GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTaskManager};
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
    return;
  }

  // Check tab bar touches
  if (ty >= 30 && ty < 55) {
    int tabW = LCD_WIDTH / TM_TAB_COUNT;
    int clickedTab = tx / tabW;
    if (clickedTab >= 0 && clickedTab < TM_TAB_COUNT) {
      taskManagerTab = clickedTab;
      uiClick();
      GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTaskManager};
      xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
      return;
    }
  }
  
  // Check control bar (exit)
  if (ty >= BAR_Y) {
    uiClick();
    currentMode = MODE_LAUNCHER;
    GpuTaskMessage msg = {GPU_CMD_DRAW_LAUNCHER};
    xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
    return;
  }
  
  // Otherwise just refresh display
  GpuTaskMessage msg = {GPU_CMD_CUSTOM_DRAW, {}, nullptr, nullptr, drawTaskManager};
  xQueueSend(gpuQueue, &msg, pdMS_TO_TICKS(10));
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200); delay(300);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  i2cScan(); loadSettings();
  pinMode(BOOT_PIN, INPUT_PULLUP); // Boot button for screenshots
  initSDCard(); // SD card (SDMMC 1-bit)
  prefs.begin("wave_os", true);
  bool isHibernate = prefs.getBool("hibernate", false);
  if (isHibernate) {
    currentMode = (AppMode)prefs.getInt("hib_mode", MODE_LAUNCHER);
    cameraMode = (CameraMode)prefs.getInt("hib_cam", CAM_MODE_NONE);
    prefs.end(); prefs.begin("wave_os", false); prefs.putBool("hibernate", false); prefs.end();
  } else { currentMode = MODE_SPLASH; prefs.end(); }
  if (wifiTargetSSID.length() > 0) { WiFi.mode(WIFI_STA); WiFi.begin(wifiTargetSSID.c_str(), wifiPassword.c_str()); Serial.println("Attempting WiFi connection at boot..."); ntpSyncNow(); }
  imuInit(); lcdHardwareReset(); delay(300); ft6336AutoDetect();
  pinMode(PIN_LCD_DC, OUTPUT); lcdSPI.begin(PIN_LCD_SCLK, PIN_LCD_MISO, PIN_LCD_MOSI, -1); st7796Init();
  // Allocate PSRAM framebuffer for screenshots (LCD MISO not connected, can't read GRAM)
  lcdFB = (uint16_t*)heap_caps_malloc((size_t)LCD_WIDTH * LCD_HEIGHT * 2, MALLOC_CAP_SPIRAM);
  Serial.printf("PSRAM total: %d bytes, free: %d bytes\n", ESP.getPsramSize(), ESP.getFreePsram());
  if (lcdFB) { Serial.println("LCD framebuffer allocated in PSRAM"); memset(lcdFB, 0, (size_t)LCD_WIDTH * LCD_HEIGHT * 2); }
  else { Serial.println("WARNING: framebuffer alloc failed - PSRAM may be disabled in board config (Tools > PSRAM: OPI/QSPI PSRAM, not Disabled)"); }
  
  // ---------- Multi-Core Task Setup ----------
  Serial.println("Initializing multi-core task system...");
  
  // Create GPU command queue
  gpuQueue = xQueueCreate(20, sizeof(GpuTaskMessage));
  if (gpuQueue == nullptr) {
    Serial.println("ERROR: Failed to create GPU queue!");
    return;
  }
  
  // Create LCD mutex for thread-safe display access
  lcdMutex = xSemaphoreCreateMutex();
  if (lcdMutex == nullptr) {
    Serial.println("ERROR: Failed to create LCD mutex!");
    return;
  }
  
  // Create GPU task on Core 1 (pinned to core 1 for graphics only)
  BaseType_t gpuResult = xTaskCreatePinnedToCore(
    gpuTask,           // Task function
    "GPU_Task",         // Task name
    8192,              // Stack size
    nullptr,           // Parameters
    2,                 // Priority (higher than CPU task)
    &gpuTaskHandle,    // Task handle
    1                  // Core 1 (GPU)
  );
  
  if (gpuResult != pdPASS) {
    Serial.println("ERROR: Failed to create GPU task!");
    return;
  }
  
  // Create CPU task on Core 0 (pinned to core 0 for all other operations)
  BaseType_t cpuResult = xTaskCreatePinnedToCore(
    cpuTask,           // Task function
    "CPU_Task",        // Task name
    8192,              // Stack size
    nullptr,           // Parameters
    1,                 // Priority (lower than GPU task)
    &cpuTaskHandle,    // Task handle
    0                  // Core 0 (CPU)
  );
  
  if (cpuResult != pdPASS) {
    Serial.println("ERROR: Failed to create CPU task!");
    return;
  }
  
  Serial.println("Multi-core system initialized successfully!");
  Serial.println("Core 0 (CPU): Input, WiFi, Camera, Filesystem");
  Serial.println("Core 1 (GPU): Graphics, Display, UI");
  
  // Initialize CPU statistics
  lastCpuStatsUpdate = millis();
  
  pinMode(PIN_LCD_BL, OUTPUT); analogWrite(PIN_LCD_BL, settingBrightness * 255 / 100);
  soundInit(); lastActivityTime = millis();
  if (currentMode == MODE_SPLASH) { splashStartTime = millis(); drawSplash(); }
  else {
    switch (currentMode) {
      case MODE_LAUNCHER: drawLauncher(); break;
      case MODE_SETTINGS: drawSettingsMain(); break;
      case MODE_SETTINGS_PRIVACY: drawSettingsPrivacy(); break;
      case MODE_SETTINGS_PREFS: drawSettingsPrefs(); break;
      case MODE_SETTINGS_SOUND: drawSettingsSound(); break;
      case MODE_SETTINGS_TIME: drawSettingsTime(); break;
      case MODE_GMAIL_SIGNIN: drawGmailSignin(); break;
      case MODE_EMAIL_INBOX: drawEmailInbox(); break;
      case MODE_EMAIL_COMPOSE: drawEmailCompose(); break;
      case MODE_TASK_MANAGER: drawTaskManager(); break;
      case MODE_SETTINGS_SAVER: drawSettingsSaver(); break;
      case MODE_WIFI: drawWiFiScreen(); break;
      case MODE_NOTEPAD: drawNotepad(); break;
      case MODE_SKETCHPAD: drawSketchPad(); break;
      case MODE_RACING: initRacing(); break;
      case MODE_CAMERA: case MODE_CAMERA_DIALOG:
        if (cameraMode != CAM_MODE_NONE) { initCamera(); drawControlBar(); if (cameraMode == CAM_MODE_DIAGNOSTIC) drawDiagnosticOverlay(); drawShutterButton(); }
        else { currentMode = MODE_LAUNCHER; drawLauncher(); } break;
      default: currentMode = MODE_LAUNCHER; drawLauncher(); break;
    }
  }

  // Note: The main loop() is now empty as all processing happens in the dedicated CPU and GPU tasks
  // This keeps the main loop available for any Arduino framework requirements
}

// Minimal loop() for Arduino framework compatibility
// All actual processing happens in the dedicated CPU (Core 0) and GPU (Core 1) tasks
void loop() {
  delay(1000); // Keep the loop alive for Arduino framework requirements
}
