/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_TRUST_SOURCE_COUNT 5
#define SENSOR_TRUST_EVENT_CAPACITY 16
#define SENSOR_TRUST_LOST_THRESHOLD 0.25f
#define SENSOR_TRUST_RESTORED_THRESHOLD 0.75f
#define SENSOR_TRUST_DEBOUNCE_MS 1000
#define SENSOR_TRUST_DISPLAY_MS 3000

typedef struct sensorTrustInput_s {
    bool active;
    bool valid;
    float confidence;
} sensorTrustInput_t;

typedef struct sensorTrustState_s {
    bool known;
    bool trusted;
    bool pending;
    uint32_t pendingSinceMs;
} sensorTrustState_t;

typedef struct sensorTrustEvent_s {
    uint8_t source;
    bool restored;
} sensorTrustEvent_t;

typedef struct sensorTrustMonitor_s {
    sensorTrustState_t source[SENSOR_TRUST_SOURCE_COUNT];
    sensorTrustEvent_t events[SENSOR_TRUST_EVENT_CAPACITY];
    uint8_t count;
    uint32_t visibleMs;
    uint32_t lastDrawMs;
    bool wasVisible;
} sensorTrustMonitor_t;

void sensorTrustUpdate(sensorTrustMonitor_t *monitor, const sensorTrustInput_t inputs[SENSOR_TRUST_SOURCE_COUNT], uint32_t nowMs);
// Peek accounts for the previous frame. Events expire only after actual display;
// the caller marks a frame visible only when this event wins message rotation.
const sensorTrustEvent_t *sensorTrustPeek(sensorTrustMonitor_t *monitor, uint32_t nowMs);
void sensorTrustSetVisible(sensorTrustMonitor_t *monitor, bool visible);
