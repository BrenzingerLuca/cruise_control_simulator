#include "car.h"
#include "pid.h"
#include "csv.h"
#include "input_validator.h"
#include "visualizer.h"


int main() {

    CSV csv;
    //Initialising the vehicle parameters for the simplified dynamic model
    const double m_car{1000.0}; //Mass of the car in kg
    const double friction_coefficient{50.0}; //N * s/m
    const double sim_duration{500.0}; //Determines how long the simulation will run in s
    const double time_step{0.2}; //s

    //Initialise the car class with the given parameters
    CAR Car(m_car, friction_coefficient);

    //Declaring starting and goal velocity. Will later be inputed by the user
    double starting_velocity{0.0};
    double goal_velocity{0.0};

    //Set initial conditions for state variables:
    double error_prev{0.0};
    double integral_k{0.0};

    InputValidator inputvalidator;
    // Get user defined velocities
    inputvalidator.double_check("Enter starting velocity (m/s) ", starting_velocity);
    inputvalidator.double_check("Enter goal velocity (m/s) ", goal_velocity);

    //Create a vector that saves the velocity values during the simulation
    std::vector<double> v_velocity = {};

    //Create PID controller object and configure parameters
    PID pid_controller;
    pid_controller.configureParameters();
    pid_controller.displayParameters();
  

    //Set starting conditions. (Time=0 and starting velocity)
    double elapsed_time{0};
    double current_vel = starting_velocity;
    
    //Main simulation loop: apply PID control and update vehicle dynamics for each timestep
    while(elapsed_time <= sim_duration)
    {
      //Save the current velocity in a vector for later visualization
      v_velocity.push_back(current_vel);

      //Compute PID control signal
      double u= pid_controller.computeU(time_step, goal_velocity, current_vel, error_prev, integral_k);

      //Calculate the acc using the model of the simplified system equation of the car 
      double current_acc = Car.calculate_acc(m_car, friction_coefficient, current_vel, u);
  
      //Calculate the starting velocity of the next timestep using Euler solver
      current_vel = Car.update_vel(current_vel, current_acc, time_step);
    
      //Advance simulation time
      elapsed_time = elapsed_time + time_step;
    }

    //Uncomment this if you want the velocities printed in the console
    // for (auto value : v_velocity){
    //   std::cout << value << std::endl;
    // }

    // Display the simulation in terminal
    Visualizer visualizer;
    visualizer.velocity_over_time(v_velocity);
    
    //Export simulation data to CSV file
    csv.write_to_csv(time_step, v_velocity, "my_cruise.csv");

    return 0;
}