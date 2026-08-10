#include <iostream>
#include <string>

int main() {
    std::cout << "Wave OS - Shop Download Application" << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "Welcome to Wave OS!" << std::endl;
    
    std::string userInput;
    bool running = true;
    
    while (running) {
        std::cout << "\nOptions:" << std::endl;
        std::cout << "1. Browse Shop" << std::endl;
        std::cout << "2. Download Item" << std::endl;
        std::cout << "3. View Downloads" << std::endl;
        std::cout << "4. Exit" << std::endl;
        std::cout << "Enter your choice: ";
        
        std::getline(std::cin, userInput);
        
        if (userInput == "1") {
            std::cout << "Opening shop..." << std::endl;
        } else if (userInput == "2") {
            std::cout << "Downloading item..." << std::endl;
        } else if (userInput == "3") {
            std::cout << "Viewing downloads..." << std::endl;
        } else if (userInput == "4") {
            std::cout << "Thank you for using Wave OS!" << std::endl;
            running = false;
        } else {
            std::cout << "Invalid option. Please try again." << std::endl;
        }
    }
    
    return 0;
}
