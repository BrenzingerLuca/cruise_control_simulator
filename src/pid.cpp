#include "cruise_control/pid.h"
#include <algorithm>

PID::PID(double k_p, double k_i, double k_d, double max_engine_force)
    : _k_p{k_p}, _k_i{k_i}, _k_d{k_d}, _max_engine_force{max_engine_force} {}

double PID::computeU(double time_step, double goal_velocity, double current_velocity) {

    // error e(t) for the control signal u(t):
    double error = goal_velocity - current_velocity;

    // u(t) from continous PID formula into discrete timestep formula: u(t)=P+Ki*I+Kd*D=P+I+D
    //P-Term
    double P = _k_p * error;

    //I-Term (time diskret: I[k]= I[k-1]+e[k]*dt)
    // Kept as a candidate until we know whether the output actually
    // saturates below (anti-windup).
    double integral_candidate = _integral_k + (error * time_step);
    double I = _k_i * integral_candidate;

    //D-Term (time diskret: D[k]=(e[k]-e[k-1])/dt)
    double derivative_k = (error - _error_prev)/time_step;
    double D = _k_d * derivative_k;
    _error_prev = error;

    //control output: u(t) in discret form
    double u_unclamped = P+I+D;

    // Actuator saturation: the engine can only deliver a limited force.
    double u = std::clamp(u_unclamped, -_max_engine_force, _max_engine_force);

    // Anti-windup via conditional integration: only commit the integral
    // update if the controller is not saturated, or if committing it would
    // pull the output back out of saturation. This stops the integral term
    // from growing unbounded while the actuator is maxed out.
    bool saturated = (u != u_unclamped);
    bool pushes_deeper_into_saturation = (u_unclamped > 0.0 && error > 0.0) || (u_unclamped < 0.0 && error < 0.0);
    if (!saturated || !pushes_deeper_into_saturation) {
        _integral_k = integral_candidate;
    }

    return u;
}
