/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <string.h>
#include "sensor_fusion.h"

static float maximum(float a, float b)
{
    return a > b ? a : b;
}

static float absolute(float a)
{
    return a < 0 ? -a : a;
}

void sensorFusionReset(sensorFusion_t *fusion)
{
    memset(fusion, 0, sizeof(*fusion));
}

bool sensorFusionUpdate(sensorFusion_t *fusion, const sensorFusionSample_t *samples,
    unsigned count, unsigned axes, uint32_t nowUs, const sensorFusionConfig_t *config)
{
    fusion->healthy = false;
    fusion->contributors = 0;
    if (!count || count > SENSOR_FUSION_MAX_SENSORS || !axes || axes > SENSOR_FUSION_AXES ||
        !isfinite(config->deviation) || config->deviation <= 0 || !config->maxAgeUs ||
        !isfinite(config->recoveryTimeSeconds) || config->recoveryTimeSeconds <= 0) {
        for (unsigned i = 0; i < SENSOR_FUSION_MAX_SENSORS; i++) {
            fusion->member[i].usable = false;
            fusion->member[i].confidence = 0;
        }
        return false;
    }

    float innovation[SENSOR_FUSION_MAX_SENSORS] = {0};
    bool expired[SENSOR_FUSION_MAX_SENSORS] = {false};
    unsigned usable = 0;
    for (unsigned i = 0; i < count; i++) {
        sensorFusionMember_t *member = &fusion->member[i];
        const sensorFusionSample_t *sample = &samples[i];
        const int32_t age = (int32_t)(nowUs - sample->timestampUs);
        member->historyExpired |= member->initialized &&
            (uint32_t)(nowUs - member->processedAtUs) > config->maxAgeUs;
        expired[i] = member->historyExpired;
        member->usable = sample->valid && age >= 0 && (uint32_t)age <= config->maxAgeUs &&
            (!member->initialized || expired[i] || (int32_t)(sample->timestampUs - member->processedAtUs) >= 0);
        for (unsigned axis = 0; axis < axes; axis++) {
            member->usable = member->usable && isfinite(sample->value[axis]);
        }
        if (!member->usable) {
            member->confidence = 0;
            continue;
        }
        usable++;
        if (expired[i]) {
            member->confidence = 0;
        }
        if (member->initialized && !expired[i]) {
            const float dt = (uint32_t)(sample->timestampUs - member->acceptedAtUs) * 1e-6f;
            // Bound extrapolation after a prolonged failure; a frozen trend is not truth.
            const float horizon = fminf(dt, config->maxAgeUs * 1e-6f);
            for (unsigned axis = 0; axis < axes; axis++) {
                float expected = member->accepted[axis] + member->velocity[axis] * horizon;
                if (!isfinite(expected)) {
                    expected = member->accepted[axis];
                }
                innovation[i] = maximum(innovation[i], absolute(sample->value[axis] - expected));
            }
        }
    }

    for (unsigned i = 0; i < count; i++) {
        sensorFusionMember_t *member = &fusion->member[i];
        const sensorFusionSample_t *sample = &samples[i];
        if (!member->usable || (member->initialized && !expired[i] && member->processedAtUs == sample->timestampUs)) {
            continue;
        }
        float disagreement = 0;
        for (unsigned axis = 0; axis < axes; axis++) {
            float values[SENSOR_FUSION_MAX_SENSORS];
            unsigned n = 0;
            for (unsigned j = 0; j < count; j++) {
                if (fusion->member[j].usable) {
                    values[n++] = samples[j].value[axis];
                }
            }
            for (unsigned j = 1; j < n; j++) {
                for (unsigned k = j; k && values[k] < values[k - 1]; k--) {
                    float tmp = values[k]; values[k] = values[k - 1]; values[k - 1] = tmp;
                }
            }
            // Median for 3 sources; midpoint for 2 cannot identify the faulty source.
            const float center = n & 1 ? values[n / 2] : (values[0] * 0.5f + values[1] * 0.5f);
            disagreement = maximum(disagreement, absolute(sample->value[axis] - center));
        }
        float score = 0;
        if (usable >= 3) {
            score = disagreement / config->deviation;
        } else if (usable == 2 && disagreement > config->deviation * 0.5f) {
            // For two sources, temporal evidence determines whose weight falls.
            // With no temporal evidence, lower both equally and expose ambiguity.
            score = member->initialized && !expired[i] ? innovation[i] / config->deviation : disagreement / config->deviation;
        } else if (usable == 1 && member->initialized) {
            score = innovation[i] / config->deviation;
        }
        // A coherent common motion is accepted even when its temporal innovation is large.
        const bool consistent = score <= 1.0f;
        const float target = consistent ? 1.0f : 1.0f / (score * score);
        float dt = member->initialized ? (uint32_t)(sample->timestampUs - member->processedAtUs) * 1e-6f : 0;
        dt = fminf(dt, config->maxAgeUs * 1e-6f);
        if (!member->initialized) {
            member->confidence = target;
        } else {
            if (target < member->confidence) {
                member->confidence = target; // Do not let a single large spike reach the controller.
            } else {
                const float alpha = dt / (config->recoveryTimeSeconds + dt);
                member->confidence += alpha * (target - member->confidence);
            }
        }
        if (consistent || !member->initialized) {
            if (member->initialized && consistent && !expired[i]) {
                const float historyDt = (uint32_t)(sample->timestampUs - member->acceptedAtUs) * 1e-6f;
                if (historyDt > 0 && historyDt <= config->maxAgeUs * 1e-6f) {
                    for (unsigned axis = 0; axis < axes; axis++) {
                        const float rate = (sample->value[axis] - member->accepted[axis]) / historyDt;
                        if (isfinite(rate)) {
                            member->velocity[axis] += 0.1f * (rate - member->velocity[axis]);
                        }
                        if (!isfinite(rate) || !isfinite(member->velocity[axis])) {
                            member->velocity[axis] = 0;
                        }
                    }
                } else {
                    memset(member->velocity, 0, sizeof(member->velocity));
                }
            }
            if (expired[i]) {
                memset(member->velocity, 0, sizeof(member->velocity));
            }
            memcpy(member->accepted, sample->value, axes * sizeof(float));
            member->acceptedAtUs = sample->timestampUs;
        }
        member->processedAtUs = sample->timestampUs;
        member->initialized = true;
        member->historyExpired = false;
    }

    float sum = 0;
    float output[SENSOR_FUSION_AXES] = {0};
    for (unsigned i = 0; i < count; i++) {
        const sensorFusionMember_t *member = &fusion->member[i];
        if (!member->usable || member->confidence < SENSOR_FUSION_MIN_CONFIDENCE) {
            continue;
        }
        sum += member->confidence;
        fusion->contributors++;
        for (unsigned axis = 0; axis < axes; axis++) {
            output[axis] += samples[i].value[axis] * member->confidence;
        }
    }
    if (sum <= 1e-6f || !isfinite(sum)) {
        return false;
    }
    for (unsigned axis = 0; axis < axes; axis++) {
        output[axis] /= sum;
        if (!isfinite(output[axis])) {
            return false;
        }
    }
    memcpy(fusion->output, output, axes * sizeof(float));
    fusion->healthy = true;
    return true;
}
