#include <gtest/gtest.h>
#include "cruise_control/metrics.h"
#include "cruise_control/simulation.h"

// Test 1: Overshoot as a percentage of the goal
TEST(MetricsTest, CalculatesOvershootPercent) {
    // Peak of 22.0 against a goal of 20.0 -> 10% overshoot.
    std::vector<double> velocities = {0.0, 10.0, 22.0, 21.0, 20.0, 20.0};

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 1.0);

    EXPECT_NEAR(result.overshoot_percent, 10.0, 1e-9);
}

// Test 2: No overshoot when the goal is approached from below
TEST(MetricsTest, ReportsZeroOvershootWhenGoalIsApproachedFromBelow) {
    std::vector<double> velocities = {0.0, 5.0, 10.0, 15.0, 20.0};

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 1.0);

    EXPECT_DOUBLE_EQ(result.overshoot_percent, 0.0);
}

// Test 3: Settling time is the moment the response last leaves the tolerance band
TEST(MetricsTest, CalculatesSettlingTime) {
    // Goal=20.0, settling_band=2% -> tolerance=0.4.
    // Last sample outside the tolerance band is index 3 (20.5), so the
    // response settles right after it: (3 + 1) * dt = 4.0.
    std::vector<double> velocities = {0.0, 10.0, 24.0, 20.5, 20.2, 20.1};

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 1.0, 0.02);

    EXPECT_DOUBLE_EQ(result.settling_time, 4.0);
}

// Test 4: Settling time is zero when the response never leaves the band
TEST(MetricsTest, ReportsZeroSettlingTimeWhenAlwaysWithinBand) {
    std::vector<double> velocities = {20.0, 20.0, 20.0};

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 0.5);

    EXPECT_DOUBLE_EQ(result.settling_time, 0.0);
}

// Test 5: Steady-state error is the absolute offset of the final sample
TEST(MetricsTest, CalculatesSteadyStateError) {
    std::vector<double> velocities = {0.0, 10.0, 19.5};

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 1.0);

    EXPECT_DOUBLE_EQ(result.steady_state_error, 0.5);
}

// Test 6: An empty trace yields zeroed-out metrics instead of undefined behavior
TEST(MetricsTest, ReturnsZeroedMetricsForEmptyInput) {
    std::vector<double> velocities;

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(velocities, 20.0, 1.0);

    EXPECT_DOUBLE_EQ(result.overshoot_percent, 0.0);
    EXPECT_DOUBLE_EQ(result.settling_time, 0.0);
    EXPECT_DOUBLE_EQ(result.steady_state_error, 0.0);
}

// Test 7: Integration check - metrics on a real, converged simulation run
TEST(MetricsTest, EvaluatesRealisticSimulationRun) {
    Simulation sim(0.1, 500.0, 50.0, 1000.0, 0.4, 0.015, 0.0, 15.0, 2.0, 1.5, 6000.0, 20.0, 0.0);
    sim.run();

    Metrics metrics;
    PerformanceMetrics result = metrics.evaluate(sim.get_results(), 20.0, 0.1);

    EXPECT_NEAR(result.steady_state_error, 0.0, 0.1);
    EXPECT_LT(result.settling_time, 500.0);
}
