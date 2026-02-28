#include "cruise_control/pid.h"

PID::PID(double k_p, double k_i, double k_d): _k_p{k_p}, _k_i{k_i}, _k_d{k_d} {} 

double PID::computeU(double time_step, double goal_velocity, double current_velocity) {

    // error e(t) for the control signal u(t):
    double error = goal_velocity - current_velocity;
    
    // u(t) from continous PID formula into discrete timestep formula: u(t)=P+Ki*I+Kd*D=P+I+D
    //P-Term
    double P = _k_p * error;

    //I-Term (time diskret: I[k]= I[k-1]+e[k]*dt)
    _integral_k = _integral_k + (error * time_step);
    double I = _k_i * _integral_k;

    //D-Term (time diskret: D[k]=(e[k]-e[k-1])/dt)
    double derivative_k = (error - _error_prev)/time_step;
    double D = _k_d * derivative_k; 
    _error_prev = error;

    //control output: u(t) in discret form
    double u = P+I+D;
    return u;
}
