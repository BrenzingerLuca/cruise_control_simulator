#ifndef CAR_H
#define CAR_H
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class Car {
private:
    const double _car_mass;
    const double _friction_coefficient;

public:
    Car(double mass, double friction) : _car_mass{mass}, _friction_coefficient{friction} {}

    double calculate_acc(double current_vel, double engine_force);
    double update_vel(double current_vel, double current_acc, double time_step);
};

#endif