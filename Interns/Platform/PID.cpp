//file PID.cpp
#include "PID.h"
#include <math.h>

PID::PID(float error, float kp, float ki, float kd, float max_output, float integral_error_range, float max_integral, bool clear_integral_windup, float settle_error, float settle_time, float timeout, float dt):
        previous_error(error),
        kp(kp),
        ki(ki * dt * 0.001),
        kd(kd / dt * 1000),
        integral_error_range(integral_error_range),
        max_integral(max_integral >= 0 ? max_integral * (1000.f / dt) : max_output / this->ki),
        clear_integral_windup(clear_integral_windup),
        derivative_alpha(dt / ((1000.f / dt) * 12 + dt)),
        max_output(max_output),
        settle({settle_error, settle_time, timeout}, dt)
{}

PID::PID(float kp, float ki, float kd, float max_output, float integral_error_range, float max_integral, bool clear_integral_windup, float settle_error, float settle_time, float timeout, float dt):
        PID(0, kp, ki, kd, max_output, integral_error_range, max_integral, clear_integral_windup, settle_error, settle_time, timeout, dt)
{}

PID::PID(float error, const PIDConfig& config, float dt):
        PID(error, config, {}, dt)
{}

PID::PID(const PIDConfig& config, float dt):
        PID(0, config, dt)
{}

PID::PID(float error, const PIDConfig& config, const SettleConfig& settle, float dt):
        PID(error, config.kp, config.ki, config.kd, config.max_output, config.integral_error_range, config.max_integral, config.reset_integral_windup, settle.settle_error, settle.settle_time, settle.timeout, dt)
{}

PID::PID(const PIDConfig& config, const SettleConfig& settle, float dt):
        PID(0, config, settle, dt)
{}

float PID::compute(float error) {
    if (this->clear_integral_windup && ((error > 0) != (this->previous_error > 0) || error == 0))
        this->integral = 0;
    if (fabsf(error) <= this->integral_error_range)
        this->integral = clamp(this->integral + error, -this->max_integral, this->max_integral);

    this->filtered_derivative += this->derivative_alpha * ((error - this->previous_error) - this->filtered_derivative);

    this->output = this->kp * error + this->ki * this->integral + this->kd * (this->filtered_derivative);

    this->settle.update(error);
    this->previous_error = error;

    return clamp(this->output, -this->max_output, this->max_output);
}

float PID::compute(float error, float error_derivative) {
    if (this->clear_integral_windup && ((error > 0) != (this->previous_error > 0) || error == 0))
        this->integral = 0;
    if (fabsf(error) <= this->integral_error_range)
        this->integral = clamp(this->integral + error, -this->max_integral, this->max_integral);

    this->filtered_derivative += this->derivative_alpha * (error_derivative - this->filtered_derivative);
    this->previous_error = error;

    this->output = this->kp * error + this->ki * this->integral + this->kd * this->filtered_derivative;
    this->settle.update(error);

    return clamp(this->output, -this->max_output, this->max_output);
}

/**
 * @brief Check if the PID controller has reached the setpoint or was terminated early
 * @param current: the motor current
 * @return true if the PID controller has reached the setpoint or was terminated early, false otherwise
 */
bool PID::is_done() {
    return this->settle.is_exit();
}

/**
 * @brief Check if the PID controller has been terminated early
 * @param current: the motor current
 * @return true if the PID controller has been terminated early, false otherwise
 */
bool PID::is_terminated() {
    return this->settle.is_timeout();
}

/**
 * @brief Check if the PID controller has settled
 * @return true if the PID controller has settled, false otherwise
 */
bool PID::is_settled() {
    return this->settle.is_settled();
}

PIDConfig PID::to_PIDConfig() const {
    return {this->kp, this->ki, this->kd, this->integral_error_range};
}

void PID::configure(const PIDConfig& config) {
    this->kp = config.kp;
    this->ki = config.ki;
    this->kd = config.kd;
    this->integral_error_range = config.integral_error_range;
}