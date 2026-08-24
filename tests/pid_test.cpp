#include <gtest/gtest.h>
#include "cruise_control/pid.h"

// Large sentinel actuator limit used by the legacy tests below so that
// saturation never kicks in and the original expected values still hold.
constexpr double NO_SATURATION = 1e6;

// Test-Case: P!=0, I=0, D=0
TEST(PIDControllerTest, CalculatesCorrectPOutput) {

    // Create a PID-Controller with Kp=5.0, Ki=0, Kd=0
    PID pid(5.0, 0.0, 0.0, NO_SATURATION);
    double target = 20.0;
    double current = 10.0;
    double dt = 0.1;

    // Apply one calculation of u
    double u = pid.computeU(dt, target, current);

    // Error is 20-10 = 10. u has to be 5.0 * 10 = 50.0.
    // Use EXPECT_DOUBLE_EQ because we dont need exact the same bit values
    EXPECT_DOUBLE_EQ(u, 50.0);
}

// Test-Case: P=0, I!=0, D=0
TEST(PIDControllerTest, CalculatesCorrectIOutput) {

    // Create a PID-Controller with Kp=0.0, Ki=2.0, Kd=0
    PID pid(0.0, 2.0, 0.0, NO_SATURATION);
    double target = 20.0;
    double current = 10.0;
    double dt = 0.1;

    double u_1 = pid.computeU(dt, target, current);
    double u_2 = pid.computeU(dt, target, current);

    // Error stays the same so u2 should be greater then u1
    EXPECT_GT(u_2, u_1);

    // Change the sign of the error u should get smaller
    current = 30;
    double u_3 = pid.computeU(dt, target, current);

    EXPECT_GT(u_2, u_3);
}

// Test-Case: P=0, I=0, D!=0
TEST(PIDControllerTest, CalculatesCorrectDOutput) {

    // Create a PID-Controller with Kp=0.0, Ki=0.0, Kd=2.0
    PID pid(0.0, 0.0, 0.1, NO_SATURATION);
    double target = 20.0;
    double current = 10.0;
    double dt = 0.1;

    double u_1 = pid.computeU(dt, target, current);

    // Calculated value with the configuration
    EXPECT_DOUBLE_EQ(u_1, 10.0);

    double u_2 = pid.computeU(dt, target, current);

    // Because the error stays the same we expect u = 0
    EXPECT_DOUBLE_EQ(u_2, 0.0);
}

// Test-Case: Goal-Velocity = Current-Velocity
TEST(PIDControllerTest, Idle) {

    // Create a PID-Controller with Kp=0.0, Ki=0.0, Kd=2.0
    PID pid(1.0, 1.0, 1.0, NO_SATURATION);
    double target = 10.0;
    double current = 10.0;
    double dt = 0.1;

    double u = pid.computeU(dt, target, current);

    // Calculated value with the configuration
    EXPECT_DOUBLE_EQ(u, 0.0);
}

// Test-Case: Output is clamped to the positive actuator limit
TEST(PIDControllerTest, ClampsOutputToMaxEngineForce) {

    PID pid(10.0, 0.0, 0.0, 50.0);
    double target = 100.0;
    double current = 0.0;
    double dt = 0.1;

    // Unclamped output would be 10.0 * 100 = 1000, but the actuator
    // saturates at 50.0.
    double u = pid.computeU(dt, target, current);
    EXPECT_DOUBLE_EQ(u, 50.0);
}

// Test-Case: Output is clamped to the negative actuator limit
TEST(PIDControllerTest, ClampsOutputToNegativeMaxEngineForce) {

    PID pid(10.0, 0.0, 0.0, 50.0);
    double target = -100.0;
    double current = 0.0;
    double dt = 0.1;

    double u = pid.computeU(dt, target, current);
    EXPECT_DOUBLE_EQ(u, -50.0);
}

// Test-Case: Anti-windup prevents the integral term from running away
// while the actuator is saturated.
TEST(PIDControllerTest, PreventsIntegralWindupUnderSaturation) {

    // Small saturation limit combined with a large integral gain drives the
    // controller into saturation almost immediately.
    PID pid(1.0, 5.0, 0.0, 10.0);
    double dt = 0.1;

    // Keep a large, constant positive error for many steps.
    for (int i = 0; i < 50; ++i) {
        double u = pid.computeU(dt, 100.0, 0.0);
        EXPECT_LE(u, 10.0);
    }

    // The error suddenly disappears. Without anti-windup, the accumulated
    // integral would keep pushing the output towards saturation for a long
    // time. With conditional integration, the output should drop
    // immediately since the integral was frozen while saturated.
    double u_after_reversal = pid.computeU(dt, 0.0, 0.0);
    EXPECT_LT(u_after_reversal, 10.0);
}
