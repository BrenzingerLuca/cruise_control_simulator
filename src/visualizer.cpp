#include "cruise_control/visualizer.h"
#include "third_party/matplotlibcpp.h"
#include <vector>
#include <string>
#include <filesystem>

namespace plt = matplotlibcpp;

void Visualizer::plot_results(const std::vector<double>& velocities, 
                              double time_step, 
                              const std::string& filename) 
{
    if (velocities.empty()) return;

    // Create the time vector
    std::vector<double> x(velocities.size());
    for (size_t i = 0; i < velocities.size(); i++) {
        x[i] = i * time_step;
    }

    // Prepare figure
    plt::figure_size(1200, 780);

    // Plotting the velocity over time in b- = blue
    plt::plot(x, velocities, "b-"); 
    
    // Setting the last value as a reference line to see the steady state value
    std::vector<double> goal(velocities.size(), velocities.back());
    plt::plot(x, goal, "r--");

    // Setting labels. Try Catch is used here because some functionalities dont exist on WSL
    try {
        plt::title("Cruise Control Step Response");
        plt::xlabel("Time [s]");
        plt::ylabel("Velocity [m/s]");
        plt::grid(true);
    } catch (...) {
        std::cerr << "Warning: Could not set labels, but continuing..." << std::endl;
    }

    // Saving the png in the data folder
    std::filesystem::path project_root = PROJECT_ROOT_DIR;
    std::filesystem::path output_path = project_root / "data" / (filename + ".png");
    std::filesystem::create_directories(project_root / "data");

    std::cout << "Saving plot to: " << output_path << " ..." << std::endl;
    plt::save(output_path.string());
    std::cout << "Plot saved!" << std::endl;

    plt::show;
}