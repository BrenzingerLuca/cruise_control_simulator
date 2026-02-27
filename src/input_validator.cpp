#include "input_validator.h"

// check if the input is legit
bool InputValidator::double_check( const std::string& prompt, double& value ) {

        while (true) {

            // Display output stream
            std::cout << prompt << std::endl;

            // Get input stream
            std::cin >> value;
            
            // check input stream
            if(!std::cin.fail()) {
                return true;
            }

            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        }
    }