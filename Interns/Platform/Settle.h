// settle.h

#pragma once

#include "SettleConfig.h" // or whatever the include was called

class Settle {
public:
    Settle() = default;
    Settle(const SettleConfig& config, float dt = 10.f);
    
    void update(float error);

    bool is_settled() const;
    bool is_timeout() const;
    bool is_exit() const;

    float get_time_spent_running() const;
    float get_time_spent_settled() const;
    float get_dt() const;

    float settle_error = 0;
    unsigned int settle_time = 0;
    unsigned int timeout = 0;

private:
    unsigned int time_spent_running = 0;
    unsigned int time_spent_settled = 0;
    unsigned int dt = 10;
};