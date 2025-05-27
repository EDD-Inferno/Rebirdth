// settle-config.h
#pragma once

float clamp(float x, float a, float b);

/**
 * @struct SettleConfig
 * @brief Configuration for settling criteria in motions.
 * 
 * - settle_error: Maximum allowable error for settling.
 * 
 * - settle_time: Time for which the error must remain below settle_error.
 * 
 * - timeout: Maximum time allowed for the motion to settle.
 */
struct SettleConfig {
    float settle_error;
    float settle_time;
    float timeout;

    /**
     * @return a copy of the config struct
     */
    SettleConfig operator()() const;

    SettleConfig& set_settle_error(float e);
    SettleConfig& set_settle_time(float t);
    SettleConfig& set_timeout(float t);
};