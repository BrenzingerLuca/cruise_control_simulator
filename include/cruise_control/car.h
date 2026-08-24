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
    const double _aero_drag_coefficient;
    const double _rolling_resistance_coefficient;
    const double _road_grade_percent;

public:
    Car(double mass, double friction, double aero_drag_coefficient,
        double rolling_resistance_coefficient, double road_grade_percent)
        : _car_mass{mass}, _friction_coefficient{friction},
          _aero_drag_coefficient{aero_drag_coefficient},
          _rolling_resistance_coefficient{rolling_resistance_coefficient},
          _road_grade_percent{road_grade_percent} {}

    double calculate_acc(double current_vel, double engine_force);
    double update_vel(double current_vel, double current_acc, double time_step);
};

#endif