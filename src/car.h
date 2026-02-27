#ifndef CAR_H
#define CAR_H
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class CAR {
private:
    double m_car;
    double friction_coefficient;

public:
    CAR(double mass, double friction) : m_car(mass), friction_coefficient(friction) {}

    double calculate_acc(double m_car, double friction_coefficient, double current_vel, double engine_force);
    double update_vel(double current_vel, double current_acc, double time_step);
};

#endif