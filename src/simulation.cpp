#include "cruise_control/simulation.h"
#include "cruise_control/car.h"
#include "cruise_control/pid.h"

Simulation::Simulation(double time_step, double sim_duration, 
                   double friction_coefficient, double m_car, 
                   double k_p, double k_i, double k_d,
                   double goal_velocity, double start_velocity)

     : _car{m_car, friction_coefficient}, 
       _controller{k_p, k_i, k_d}, 
       _time_step{time_step},
       _sim_duration{sim_duration},
       _goal_velocity{goal_velocity},
       _start_velocity{start_velocity}
    {
        // Reserve the estimated elemts for the results vector to save computation time
        std::size_t estimated_elements = static_cast<std::size_t>(sim_duration / time_step) + 1;
        _results.reserve(estimated_elements);
}

void Simulation::run(){

    //Set starting conditions. (Time=0 and starting velocity)
    double elapsed_time{0};
    double current_vel = _start_velocity;

    //Main simulation loop: apply PID control and update vehicle dynamics for each timestep
    while(elapsed_time <= _sim_duration)
    {
        //Save the current velocity in a vector for later visualization
        _results.push_back(current_vel);

        //Compute PID control signal
        double u = _controller.computeU(_time_step, _goal_velocity, current_vel);

        //Calculate the acc using the model of the simplified system equation of the car 
        double current_acc = _car.calculate_acc(current_vel, u);
    
        //Calculate the starting velocity of the next timestep using Euler solver
        current_vel = _car.update_vel(current_vel, current_acc, _time_step);
        
        //Advance simulation time
        elapsed_time = elapsed_time + _time_step;
    }
}

const std::vector<double>& Simulation::get_results() const {
    return _results;
}

