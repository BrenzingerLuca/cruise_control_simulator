#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include "cruise_control/car.h"
#include "cruise_control/pid.h"

class Simulation{
    private: 
        Car _car;
        PID _controller;
        const double _time_step;
        const double _sim_duration;
        const double _goal_velocity;
        const double _start_velocity;
        std::vector<double> _results;

    public:
        Simulation(double time_step, double sim_duration,
                   double friction_coefficient, double m_car, 
                   double k_p, double k_i, double k_d,
                   double goal_velocity, double start_velocity);

        void run();
        const std::vector<double>& get_results() const;
};

#endif