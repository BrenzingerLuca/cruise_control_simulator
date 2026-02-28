#include "cruise_control/car.h"

double Car::calculate_acc(double current_vel, double engine_force){

  /*
  Calculate the acceleration of the car using the simplyfied car model 
  Link: https://ctms.engin.umich.edu/CTMS/index.php?example=CruiseControl&section=SimulinkModeling
  */

  double current_acc = (engine_force - (_friction_coefficient * current_vel)) / _car_mass;
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





