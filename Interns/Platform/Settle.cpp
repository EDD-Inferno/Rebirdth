// settle.cpp

#include "Settle.h"
#include <math.h>

Settle::Settle(const SettleConfig& config, float dt):
        settle_error(config.settle_error), settle_time(config.settle_time), timeout(config.timeout), dt(roundf(dt)) {}

void Settle::update(float error) {
    if (fabsf(error) < this->settle_error)
        this->time_spent_settled += this->dt;
    else
        this->time_spent_settled = 0;

    this->time_spent_running += this->dt;
}

bool Settle::is_settled() const {
    return this->settle_time != 0 && this->time_spent_settled > this->settle_time;
}

bool Settle::is_timeout() const {
    return (this->timeout != 0 && this->time_spent_running > this->timeout);
}

bool Settle::is_exit() const {
    return this->is_settled() || this->is_timeout();
}

float Settle::get_time_spent_running() const {
    return this->time_spent_running;
}

float Settle::get_time_spent_settled() const {
    return this->time_spent_settled;
}

float Settle::get_dt() const {
    return this->dt;
}