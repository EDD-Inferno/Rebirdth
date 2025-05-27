// settle-config.cpp

#include "SettleConfig.h" // or whatever it's called

float clamp(float x, float a, float b) {
    return x < a ? a : (x > b ? b : x);
}


SettleConfig SettleConfig::operator()() const {
    return *this;
}

SettleConfig& SettleConfig::set_settle_error(float e) {
    this->settle_error = e;
    return *this;
}

SettleConfig& SettleConfig::set_settle_time(float t) {
    this->settle_time = t;
    return *this;
}

SettleConfig& SettleConfig::set_timeout(float t) {
    this->timeout = t;
    return *this;
}