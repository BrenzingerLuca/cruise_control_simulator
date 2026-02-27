#include "csv.h"

void CSV::write_to_csv(
    double t_step,
    const std::vector<double>& v_velocity,
    const std::string& filename) 
    {
    
    /*
    * Generates a csv (comma separated values) of (time, velocity).
    * This creates a file "cruise_control.csv" by default in the
    * current folder. We use the <fstream> header for writing to the
    * file.
    */
    
    // get project root directory
    std::filesystem::path project_root = PROJECT_ROOT_DIR;
    
    // Check for data folder and create if not existing
    if (!std::filesystem::exists(project_root / "data")) {
        std::filesystem::create_directories(project_root / "data");
    }

    // full output path
    std::filesystem::path out_path = project_root / "data" / filename;
    // open file stream for output
    std::ofstream csv_file(out_path);
    if(!csv_file) {
        std::cerr << "Error opening file " << out_path << " for writing." << std::endl;
        return;
    }

    // csv column headers
    csv_file << "time,velocity \n";

    for (size_t i = 0; i < v_velocity.size(); i++) {

        // Calculate the current timestep based on count and t_step
        double time = i * t_step;
    
        // Write to csv
        csv_file << time << "," << v_velocity[i] << "\n";
    }

    // Close csv file
    csv_file.close();
    std::cout << "Saved data to " << out_path << "\n";

}