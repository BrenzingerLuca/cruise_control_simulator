#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <vector>
#include <string>



class Visualizer {
public:
    void plot_results(const std::vector<double>& velocities,
                      double time_step,
                      const std::string& filename);
};

#endif // VISUALIZER_H