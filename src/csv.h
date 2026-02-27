#ifndef CSV_H
#define CSV_H

#ifndef PROJECT_ROOT_DIR
#error PROJECT_ROOT_DIR not defined
#endif  // PROJECT_ROOT_DIR

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include "car.h"

class CSV {
    public:
        void write_to_csv(
            double t_step,
            const std::vector<double>& v_velocity,
            const std::string& filename = "my_cruise.csv");
};


#endif // CSV_H

