#ifndef PID_H
#define PID_H
#include <string>

class PID {
private:
    double _k_p;
    double _k_i;
    double _k_d;

    double _error_prev{0.0};
    double _integral_k{0.0};
    //The values a user can enter are purposely not limited, as the user might want to experiment with these values in order to get an understanding of PID control
    //Instead, the user gets a hint on what values might seem more realistic/reasonable. 
    //However, the code is ensuring the user can only enter numerical values (instead of e.g. letters) to not crash the system
    double getNumericInput(std::string label, std::string con_interval);

public:
    PID(double k_p, double k_i, double k_d); //constructor    
    void configureParameters();
    void displayParameters(); //for ensuring correct parameters have been applied

    double computeU(double time_step, double goal_velocity, double current_velocity);
    };
#endif