#include <gtest/gtest.h>
#include "cruise_control/car.h"

// Test 1: Basic Velocity Update
TEST(CarPhysicsTest, UpdatesVelocityCorrectly) {
    Car car(1000.0, 50.0, 0.0, 0.0, 0.0);
    double v_init = 10.0;
    double acc = 2.0;
    double dt = 0.5;

    // Expected: 10.0 + (2.0 * 0.5) = 11.0
    EXPECT_DOUBLE_EQ(car.update_vel(v_init, acc, dt), 11.0);
}

// Test 2: Acceleration from standstill
TEST(CarPhysicsTest, CalculatesAccelerationFromStandstill) {
    Car car(1000.0, 50.0, 0.0, 0.0, 0.0);
    double force = 1000.0;
    double v_current = 0.0;

    // Expected: (1000 - 50*0) / 1000 = 1.0
    double acc = car.calculate_acc(v_current, force);
    EXPECT_DOUBLE_EQ(acc, 1.0);
}

// Test 3: Friction and Deceleration (Coasting)
TEST(CarPhysicsTest, CalculatesDecelerationDueToFriction) {
    Car car(1000.0, 50.0, 0.0, 0.0, 0.0);
    double force = 0.0;     // No engine force
    double v_current = 20.0; // Moving at 20 m/s

    // Expected: (0 - 50*20) / 1000 = -1.0
    double acc = car.calculate_acc(v_current, force);
    EXPECT_DOUBLE_EQ(acc, -1.0);
}

// Test 4: Zero Acceleration
TEST(CarPhysicsTest, ReachesEquilibriumAtTerminalVelocity) {
    Car car(1000.0, 50.0, 0.0, 0.0, 0.0);
    double force = 1000.0;
    double v_current = 20.0;

    // Expected: (1000 - 50*20) / 1000 = 0.0
    double acc = car.calculate_acc(v_current, force);

    // Using NEAR because floating point math can be tricky at zero
    EXPECT_NEAR(acc, 0.0, 1e-9);
}

// Test 5: Aerodynamic drag scales with the square of the velocity
TEST(CarPhysicsTest, AppliesAerodynamicDragProportionalToVelocitySquared) {
    Car car(1000.0, 0.0, 2.0, 0.0, 0.0); // aero_drag_coefficient = 2.0
    double force = 0.0;
    double v_current = 10.0;

    // F_aero = 2.0 * 10 * |10| = 200 -> acc = -200 / 1000 = -0.2
    double acc = car.calculate_acc(v_current, force);
    EXPECT_DOUBLE_EQ(acc, -0.2);
}

// Test 6: Rolling resistance opposes the current direction of travel
TEST(CarPhysicsTest, AppliesRollingResistanceOpposingMotion) {
    Car car(1000.0, 0.0, 0.0, 0.02, 0.0); // rolling_resistance_coefficient = 0.02
    double force = 0.0;
    double v_current = 10.0;

    // F_roll = 0.02 * 1000 * 9.81 = 196.2 -> acc = -196.2 / 1000 = -0.1962
    double acc = car.calculate_acc(v_current, force);
    EXPECT_NEAR(acc, -0.1962, 1e-9);
}

// Test 7: An uphill grade increases the resisting force
TEST(CarPhysicsTest, UphillGradeReducesAcceleration) {
    Car car_flat(1000.0, 0.0, 0.0, 0.0, 0.0);
    Car car_uphill(1000.0, 0.0, 0.0, 0.0, 10.0); // 10% uphill grade
    double force = 1000.0;
    double v_current = 10.0;

    double acc_flat = car_flat.calculate_acc(v_current, force);
    double acc_uphill = car_uphill.calculate_acc(v_current, force);

    // The same engine force yields less acceleration uphill than on flat ground.
    EXPECT_LT(acc_uphill, acc_flat);
}

// Test 8: A downhill grade assists acceleration
TEST(CarPhysicsTest, DownhillGradeIncreasesAcceleration) {
    Car car_flat(1000.0, 0.0, 0.0, 0.0, 0.0);
    Car car_downhill(1000.0, 0.0, 0.0, 0.0, -10.0); // 10% downhill grade
    double force = 0.0;
    double v_current = 10.0;

    double acc_flat = car_flat.calculate_acc(v_current, force);
    double acc_downhill = car_downhill.calculate_acc(v_current, force);

    // Gravity assists on a downhill grade, giving more acceleration than on flat ground.
    EXPECT_GT(acc_downhill, acc_flat);
}