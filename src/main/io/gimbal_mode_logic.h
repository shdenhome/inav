#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct gimbalModeSwitchState_s {
    uint8_t channel;
    uint8_t selected;
    uint8_t candidate;
    uint32_t candidateSinceMs;
} gimbalModeSwitchState_t;

// Codes are experimental with C-20T: do not assume they identify user presets.
static inline uint8_t gimbalModeSelect(gimbalModeSwitchState_t *state, uint8_t channel,
    bool valid, uint16_t pwm, bool center, bool headtracker, uint8_t legacyMode, uint32_t nowMs)
{
    if (center || headtracker || channel == 0 || !valid) {
        *state = (gimbalModeSwitchState_t){0, 0, 0, 0};
        return center || headtracker || channel != 0 ? 0 : legacyMode;
    }
    if (state->channel != channel) {
        *state = (gimbalModeSwitchState_t){0, 0, 0, 0};
        state->channel = channel;
    }

    // Nominal boundaries 1300/1700 us; 40 us hysteresis on either side.
    uint8_t requested = state->selected;
    if (pwm < 1260) requested = 0;
    else if (pwm > 1740) requested = 2;
    else if ((state->selected == 0 && pwm > 1340)
        || (state->selected == 2 && pwm < 1660)) requested = 1;

    if (requested != state->candidate) {
        state->candidate = requested;
        state->candidateSinceMs = nowMs;
    }
    // 150 ms stable input; loss of input above falls back immediately.
    if ((uint32_t)(nowMs - state->candidateSinceMs) >= 150) state->selected = state->candidate;
    return state->selected;
}
