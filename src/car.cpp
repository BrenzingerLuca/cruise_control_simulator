#include "car.h"

double CAR::calculate_acc(double m_car, double friction_coefficient, double current_vel, double engine_force){

  /*
  Calculate the acceleration of the car using the simplyfied car model 
  Link: https://ctms.engin.umich.edu/CTMS/index.php?example=CruiseControl&section=SimulinkModeling
  */

  double current_acc = (engine_force - (friction_coefficient * current_vel)) / m_car;
  return current_acc;
}

double CAR::update_vel(double current_vel, double current_acc, double time_step){
  /*
  Calculate the velocity for the next time step using the euler solver 
  https://en.wikipedia.org/wiki/Euler_method
  */
 
  double updated_vel = current_vel + current_acc * time_step;
  return updated_vel;
}





