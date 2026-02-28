#include "cruise_control/car.h"
#include "cruise_control/pid.h"
#include "cruise_control/csv.h"
#include "cruise_control/input_validator.h"
#include "cruise_control/visualizer.h"
#include "cruise_control/simulation.h"


int main() {

    CSV csv;
    //Initialising the vehicle parameters for the simplified dynamic model
    const double car_mass{1000.0}; //Mass of the car in kg
    const double friction_coefficient{50.0}; //N * s/m
    const double sim_duration{500.0}; //Determines how long the simulation will run in s
    const double time_step{0.2}; //s

    double kp = 50.0;
    double ki = 1.0;
    double kd = 5.0;

    //Declaring starting and goal velocity. Will later be inputed by the user
    double starting_velocity{0.0};
    double goal_velocity{0.0};

    InputValidator inputvalidator;
    // Get user defined velocities
    inputvalidator.double_check("Enter starting velocity (m/s) ", starting_velocity);
    inputvalidator.double_check("Enter goal velocity (m/s) ", goal_velocity);

    Simulation sim(time_step, sim_duration, friction_coefficient, car_mass,
                   kp, ki, kd, goal_velocity, starting_velocity);
   
    // Run the simulation
    sim.run();
    
    // Save the results vector of the simulation 
    const auto & results = sim.get_results();

    // Display the simulation in terminal
    Visualizer visualizer;
    visualizer.velocity_over_time(results);
    
    //Export simulation data to a CSV file
    csv.write_to_csv(time_step, results, "my_cruise.csv");

    return 0;
}