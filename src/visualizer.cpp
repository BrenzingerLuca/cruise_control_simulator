#include "visualizer.h"


void Visualizer::velocity_over_time( const std::vector<double>& v_velocity ) {


    std::cout << "Plotting Velocity over Time: " << std::endl;
    // Get number of values in velocity
    int n_velocities = v_velocity.size();
    if (n_velocities == 0) return;

    // Get inverval size
    int interval = std::max(1, n_velocities / 50);

    // Get new vector that contains of 50 values which show the mean
    std::vector<double> v_intervals = {};

    // Compute the mean for each interval
    for (int i = 0; i < n_velocities; i += interval) {
        double sum = 0;
        int count = 0;
        for (int j = i; j < i + interval && j < n_velocities; j++) {
            sum += v_velocity[j];
            count++;
        }
        v_intervals.push_back(sum / count);
    }


    // transform into a displayabe discrete format
    double v_max = *std::max_element(v_intervals.begin(), v_intervals.end());
    std::vector<int> stars = {};

    for (int i = 0; i < v_intervals.size(); i++) {
        stars.push_back(int((v_intervals[i] / v_max) * 20));
    }

    for (int level = 20; level > 0; level--) {
        for (int i = 0; i < stars.size(); i++) {
            // std::cout << stars[i];
            if (level == stars[i]) {
                std::cout << "*";
            }
            else {
                std::cout << " ";
            }
        }
        std::cout << "\n";
    }

}