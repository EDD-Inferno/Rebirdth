// File: PID.h
#pragma once
#include "PIDConfig.h"
#include "settle.h"


class PID {

public:

    float kp = 0;

    float ki = 0;

    float kd = 0;

    float integral_error_range = 0;



    float integral = 0;

    float previous_error = 0;

    float output = 0;

    float filtered_derivative = 0;

    float derivative_alpha = 1;

    float max_output = 12.f;

    float max_integral = -1;



    bool clear_integral_windup = true;



    Settle settle;



    PID(float error, float kp, float ki, float kd, float max_output = 12.f, float integral_error_range = 1e20f, float max_integral = -1, bool clear_integral_windup = true, float settle_error = 0, float settle_time = 0, float timeout = 0, float dt = 10.0f);

    PID(float kp, float ki, float kd, float max_output = 12.f, float integral_error_range = 1e20f, float max_integral = -1, bool clear_integral_windup = true, float settle_error = 0, float settle_time = 0, float timeout = 0, float dt = 10.0f);



    PID(float error, const PIDConfig& config, float dt = 10.0f);

    PID(float error, const PIDConfig& config, const SettleConfig& settle, float dt = 10.0f);

    PID(const PIDConfig& config, float dt = 10.0f);

    PID(const PIDConfig& config, const SettleConfig& settle, float dt = 10.0f);



    float compute(float error);

    float compute(float error, float error_derivative);



    bool is_done();

    bool is_terminated();

    bool is_settled();



    void configure(const PIDConfig& config);



    PIDConfig to_PIDConfig() const;

};