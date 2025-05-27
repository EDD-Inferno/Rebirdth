// file: PIDConfig.h
#pragma once

struct PIDConfig {
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float integral_error_range = 1e20f;
    float derivative_time_constant = 0;
    float max_integral = -1;
    float max_output = 12;
    bool reset_integral_windup = true;

    PIDConfig() = default;
    PIDConfig(float kp, float ki, float kd, float max_output = 12.f,
              float integral_error_range = 1e20f, float max_integral = -1,
              bool reset_integral_windup = true, float derivative_time_constant = 0);

    PIDConfig operator()() const;

    PIDConfig& set_kp(float val);
    PIDConfig& set_ki(float val);
    PIDConfig& set_kd(float val);
    PIDConfig& set_max_output(float val);
    PIDConfig& set_max_integral(float val);
    PIDConfig& set_integral_error_range(float range);
    PIDConfig& set_derivative_time_constant(float tau);
    PIDConfig& set_derivative_noise_frequency(float omega);
    PIDConfig& set_reset_integral_windup(bool reset);
};