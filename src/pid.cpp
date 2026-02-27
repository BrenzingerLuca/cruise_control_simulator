#include "pid.h"
#include <iostream>
#include <limits>

PID::PID(): k_p{0.0}, k_i{0.0}, k_d{0.0} {} 

double PID::getNumericInput(std::string label, std::string con_interval) {
    double val;
    while (true) {
        std::cout << "Enter the " << label << " constant\n" << "A realistic value is:" << con_interval << "\nValue: ";
        if (std::cin >> val) {
            return val; // Enter valid number
        } else {
            std::cout << "Invalid entry! Please enter a numeric value (e.g. 1.2).\n";
            std::cin.clear(); // Clear error state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear buffer
        }
    }
}

void PID::configureParameters() {
    double default_kp = 50;
    double default_ki = 1.0;
    double default_kd = 5.0;
    char constants_choice;

    std::cout << "Would you like to apply standard values for the PID controller constants? [Y/N]";
    std::cin >> constants_choice;
    if (constants_choice == 'Y' || constants_choice == 'y'){
      k_p = default_kp;
      k_i = default_ki;
      k_d = default_kd;
      std::cout << "Standard values for PID controller applied.\n";
    }
    else if (constants_choice == 'N' || constants_choice == 'n'){
        k_p = getNumericInput("proportional", "[0.5-100.0]");
        k_i = getNumericInput("integral", "[0.01-10.0]");
        k_d = getNumericInput("differential", "[0.1-20.0]");
    }
    else{
      std::cout << "Invalid entry: Please enter the letter Y or N.\n";
    }
}
void PID::displayParameters() {
    std::cout << "Current PID constants: P=" << k_p << ", I=" << k_i << ", D=" << k_d << "\n";
}
double PID::computeU(double time_step, double goal_velocity,
    double current_velocity, double &error_prev, double &integral_k) {

    //error e(t) for the control signal u(t):
    double error = goal_velocity - current_velocity;
    
    // u(t) from continous PID formula into discrete timestep formula: u(t)=P+Ki*I+Kd*D=P+I+D
    //P-Term
    double P = k_p * error;
    //I-Term (time diskret: I[k]= I[k-1]+e[k]*dt)
    integral_k = integral_k + (error * time_step);
    double I = k_i * integral_k;
    //D-Term (time diskret: D[k]=(e[k]-e[k-1])/dt)
    double derivative_k = (error - error_prev)/time_step;
    double D = k_d * derivative_k; 
    error_prev = error;

    //control output: u(t) in discret form
    double u = P+I+D;
    return u;
}
