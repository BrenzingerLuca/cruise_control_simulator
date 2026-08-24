#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

#include "cruise_control/car.h"
#include "cruise_control/pid.h"
#include "cruise_control/csv.h"
#include "cruise_control/visualizer.h"
#include "cruise_control/simulation.h"


int main() {

    // Load the yaml file
    YAML::Node config = YAML::LoadFile("../config/config.yaml"); 

    // Read the values from the config.yaml
    const double time_step = config["simulation"]["time_step"].as<double>();
    const double sim_duration = config["simulation"]["duration"].as<double>();
    
    const double car_mass = config["car"]["mass"].as<double>();
    double friction = config["car"]["friction"].as<double>();
    const double aero_drag_coefficient = config["car"]["aero_drag_coefficient"].as<double>();
    const double rolling_resistance_coefficient = config["car"]["rolling_resistance_coefficient"].as<double>();
    const double road_grade_percent = config["car"]["road_grade_percent"].as<double>();

    const double kp = config["pid"]["p"].as<double>();
    const double ki = config["pid"]["i"].as<double>();
    const double kd = config["pid"]["d"].as<double>();
    const double max_engine_force = config["pid"]["max_engine_force"].as<double>();

    const double start_vel = config["initial"]["start_velocity"].as<double>();
    const double goal_vel = config["initial"]["goal_velocity"].as<double>();

    // Setup the simulation
    Simulation sim(time_step, sim_duration, friction, car_mass,
                    aero_drag_coefficient, rolling_resistance_coefficient, road_grade_percent,
                    kp, ki, kd, max_engine_force, goal_vel, start_vel);
   
    // Run the simulation
    sim.run();
    
    // Save the results vector of the simulation 
    const auto & results = sim.get_results();

    // Create a plot of the velocitie over time in the data folder
    Visualizer visualizer;
    visualizer.plot_results(results, time_step, "sim_plot");
    
    //Export simulation data to a CSV file
    CSV csv;
    csv.write_to_csv(time_step, results, "my_cruise.csv");

    return 0;
}