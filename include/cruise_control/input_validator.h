#ifndef INPUT_VALIDATOR_H
#define INPUT_VALIDATOR_H
#include <iostream>
#include <string>
#include <limits> 


class InputValidator {
    public:
        bool double_check(
            const std::string& prompt,
            double& value);
};

#endif // INPUT_VALIDATOR_H