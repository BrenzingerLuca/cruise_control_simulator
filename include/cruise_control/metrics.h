#ifndef METRICS_H
#define METRICS_H

#include <vector>

// Standard step-response performance metrics computed from a velocity trace.
struct PerformanceMetrics {
    double overshoot_percent;
    double settling_time;
    double steady_state_error;
};

class Metrics {
public:
    PerformanceMetrics evaluate(const std::vector<double>& velocities,
                                double goal_velocity,
                                double time_step,
                                double settling_band = 0.02);
};

#endif // METRICS_H
