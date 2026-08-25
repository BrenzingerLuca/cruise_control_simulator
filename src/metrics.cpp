#include "cruise_control/metrics.h"
#include <algorithm>
#include <cmath>

PerformanceMetrics Metrics::evaluate(
    const std::vector<double>& velocities,
    double goal_velocity,
    double time_step,
    double settling_band)
    {

    PerformanceMetrics result{0.0, 0.0, 0.0};
    if (velocities.empty()) return result;

    // Overshoot: largest excursion above the goal, as a percentage of the goal.
    double peak_velocity = *std::max_element(velocities.begin(), velocities.end());
    if (peak_velocity > goal_velocity) {
        result.overshoot_percent = (peak_velocity - goal_velocity) / std::abs(goal_velocity) * 100.0;
    }

    // Settling time: the response is considered settled right after the last
    // sample that still lies outside the +/- settling_band tolerance band
    // around the goal.
    double tolerance = std::abs(goal_velocity) * settling_band;
    bool ever_out_of_band = false;
    std::size_t last_out_of_band_index = 0;

    for (std::size_t i = 0; i < velocities.size(); i++) {
        if (std::abs(velocities[i] - goal_velocity) > tolerance) {
            last_out_of_band_index = i;
            ever_out_of_band = true;
        }
    }

    result.settling_time = ever_out_of_band
        ? static_cast<double>(last_out_of_band_index + 1) * time_step
        : 0.0;

    // Steady-state error: remaining offset between the final value and the goal.
    result.steady_state_error = std::abs(velocities.back() - goal_velocity);

    return result;
}
