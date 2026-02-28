#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>


class Visualizer {
    public:
        void velocity_over_time(const std::vector<double>& v_velocity);
};

#endif // VISUALIZER_H