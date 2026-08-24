#include "cruise_control/car.h"
#include <cmath>

namespace {
    // Standard gravity, used for rolling resistance and the road-grade force.
    constexpr double GRAVITY_MPS2 = 9.81;

    // Sign of a real number: +1, -1, or 0 for exactly zero. Keeps the rolling
    // resistance force pointed opposite to the current direction of travel.
    double sign(double value) {
        if (value > 0.0) return 1.0;
        if (value < 0.0) return -1.0;
        return 0.0;
    }
}

double Car::calculate_acc(double current_vel, double engine_force){

  /*
  Calculate the acceleration of the car using the simplyfied car model
  Link: https://ctms.engin.umich.edu/CTMS/index.php?example=CruiseControl&section=SimulinkModeling
  Extended with quadratic aerodynamic drag, rolling resistance and a
  constant road grade to model uphill/downhill driving.
  */

  // Linear (viscous) friction, e.g. drivetrain losses.
  double friction_force = _friction_coefficient * current_vel;

  // Aerodynamic drag grows with the square of the velocity. Using v*|v|
  // instead of v*v keeps the force correctly signed if the car ever moves
  // backwards (e.g. rolled back by a steep uphill grade).
  double aero_drag_force = _aero_drag_coefficient * current_vel * std::abs(current_vel);

  // Rolling resistance is roughly constant in magnitude and always opposes
  // the current direction of travel.
  double rolling_resistance_force = _rolling_resistance_coefficient * _car_mass * GRAVITY_MPS2 * sign(current_vel);

  // Gravity component along the slope. A positive grade (uphill) always
  // resists forward motion; a negative grade (downhill) assists it.
  double grade_angle = std::atan(_road_grade_percent / 100.0);
  double gravity_force = _car_mass * GRAVITY_MPS2 * std::sin(grade_angle);

  double resisting_force = friction_force + aero_drag_force + rolling_resistance_force + gravity_force;
  double current_acc = (engine_force - resisting_force) / _car_mass;
  return current_acc;
}

double Car::update_vel(double current_vel, double current_acc, double time_step){
  /*
  Calculate the velocity for the next time step using the euler solver 
  https://en.wikipedia.org/wiki/Euler_method
  */
 
  double updated_vel = current_vel + current_acc * time_step;
  return updated_vel;
}





