#ifndef PID_H
#define PID_H


class PID {
private:
    const double _k_p;
    const double _k_i;
    const double _k_d;
    const double _max_engine_force;

    double _error_prev{0.0};
    double _integral_k{0.0};

public:
    PID(double k_p, double k_i, double k_d, double max_engine_force); //constructor

    double computeU(double time_step, double goal_velocity, double current_velocity);
};

#endif