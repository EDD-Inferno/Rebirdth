// file: PIDConfig.cpp
#include "PIDConfig.h"

PIDConfig::PIDConfig(float kp, float ki, float kd, float max_output, float integral_error_range, float max_integral, bool reset_integral_windup, float derivative_noise_frequency):
        kp(kp), ki(ki), kd(kd), integral_error_range(integral_error_range), max_integral(max_integral),
        derivative_time_constant(derivative_noise_frequency), max_output(max_output), reset_integral_windup(reset_integral_windup) {}

PIDConfig PIDConfig::operator()() const {
    return *this;
}

PIDConfig& PIDConfig::set_kp(float val) {
    this->kp = val;
    return *this;
}

PIDConfig& PIDConfig::set_ki(float val) {
    this->ki = val;
    return *this;
}

PIDConfig& PIDConfig::set_kd(float val) {
    this->kd = val;
    return *this;
}

PIDConfig& PIDConfig::set_integral_error_range(float range) {
    this->integral_error_range = range;
    return *this;
}


PIDConfig& PIDConfig::set_max_integral(float val) {
    this->max_integral = val;
    return *this;
}

PIDConfig& PIDConfig::set_max_output(float val) {
    this->max_output = val;
    return *this;
}

PIDConfig& PIDConfig::set_derivative_noise_frequency(float omega) {
    this->derivative_time_constant = 1.f / omega;
    return *this;
}

PIDConfig& PIDConfig::set_derivative_time_constant(float tau) {
    this->derivative_time_constant = tau;
    return *this;
}

PIDConfig& PIDConfig::set_reset_integral_windup(bool reset) {
    this->reset_integral_windup = reset;
    return *this;
}