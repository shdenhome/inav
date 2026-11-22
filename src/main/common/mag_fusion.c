/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <string.h>
#include "mag_fusion.h"

static float bound(float value, float min, float max) { return fminf(max, fmaxf(min, value)); }
static float length(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
static bool finiteVector(const float v[3]) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
static float angle(const float a[3], const float b[3])
{
    const float product = length(a) * length(b);
    if (!isfinite(product) || product < 1e-6f) return 180;
    return acosf(bound((a[0]*b[0] + a[1]*b[1] + a[2]*b[2])/product, -1, 1)) * 57.2957795f;
}
void magFusionReset(magFusion_t *fusion) { memset(fusion, 0, sizeof(*fusion)); }

bool magFusionUpdate(magFusion_t *fusion, const magFusionSample_t samples[MULTI_MAG_MAX_SOURCES],
    uint32_t nowUs, const magFusionConfig_t *config)
{
    fusion->healthy = false; fusion->contributors = 0; fusion->fallbackId = 0;
    const bool configurationValid = config->maxAgeUs && config->recoverySeconds > 0 &&
        isfinite(config->recoverySeconds) && config->agreementDegrees > 0 &&
        config->rejectDegrees > config->agreementDegrees && config->rejectDegrees < 180 &&
        config->fieldMin > 0 && config->fieldMax > config->fieldMin && isfinite(config->fieldMax);
    unsigned usable = 0;
    float magnitudes[MULTI_MAG_MAX_SOURCES] = {0};
    for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) {
        magFusionMember_t *m = &fusion->member[i];
        const magFusionSample_t *s = &samples[i];
        m->contributing = false;
        const int32_t age = (int32_t)(nowUs - s->timestampUs);
        magnitudes[i] = finiteVector(s->field) ? length(s->field) : 0;
        m->usable = configurationValid && s->id && s->valid && age >= 0 && (uint32_t)age <= config->maxAgeUs &&
            isfinite(magnitudes[i]) && magnitudes[i] > 1e-6f &&
            (!m->initialized || (int32_t)(s->timestampUs - m->processedAtUs) >= 0);
        if (!m->usable) {
            m->confidence = 0; m->quarantined = true; m->goodPending = false; m->badPending = false;
            // A read outage can include moving the aircraft or changing environment.
            // Re-learn the earth reference from fresh data rather than trapping recovery.
            m->earthReferenceValid = false;
        } else usable++;
    }
    float center[3] = {0};
    if (usable >= 3) {
        for (unsigned axis = 0; axis < 3; axis++) {
            float values[MULTI_MAG_MAX_SOURCES]; unsigned n = 0;
            for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) if (fusion->member[i].usable)
                values[n++] = samples[i].field[axis] / magnitudes[i];
            for (unsigned j = 1; j < n; j++) for (unsigned k = j; k && values[k] < values[k-1]; k--) {
                const float t = values[k]; values[k] = values[k-1]; values[k-1] = t;
            }
            center[axis] = n & 1 ? values[n/2] : (values[n/2-1] + values[n/2]) * 0.5f;
        }
    }
    for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) {
        magFusionMember_t *m = &fusion->member[i]; const magFusionSample_t *s = &samples[i];
        if (!m->usable || (m->initialized && m->processedAtUs == s->timestampUs)) continue;
        float deviation = 0;
        if (usable >= 3 && length(center) > 0.01f) {
            deviation = angle(s->field, center);
        } else if (usable == 2) {
            float pairDeviation = 0;
            for (unsigned j = 0; j < MULTI_MAG_MAX_SOURCES; j++) if (j != i && fusion->member[j].usable)
                pairDeviation = angle(s->field, samples[j].field);
            if (pairDeviation > config->agreementDegrees) {
                deviation = s->earthValid && m->earthReferenceValid && finiteVector(s->earthField) ?
                    angle(s->earthField, m->earthReference) : pairDeviation;
            }
        } else if (s->earthValid && m->earthReferenceValid && finiteVector(s->earthField)) {
            deviation = angle(s->earthField, m->earthReference);
        }
        const float norm = magnitudes[i];
        const bool hardBad = deviation > config->rejectDegrees || norm < config->fieldMin || norm > config->fieldMax;
        const bool good = deviation <= config->agreementDegrees && norm >= 512 && norm <= 1843.2f;
        float target = 1;
        if (deviation > config->agreementDegrees)
            target = fminf(target, bound((config->rejectDegrees - deviation)/(config->rejectDegrees-config->agreementDegrees), 0.01f, 1));
        if (norm < 512) target = fminf(target, bound((norm-config->fieldMin)/fmaxf(1,512-config->fieldMin),0.01f,1));
        if (norm > 1843.2f) target = fminf(target, bound((config->fieldMax-norm)/fmaxf(1,config->fieldMax-1843.2f),0.01f,1));
        float dt = m->initialized ? fminf((uint32_t)(s->timestampUs-m->processedAtUs)*1e-6f,config->maxAgeUs*1e-6f) : 0;
        if (!m->initialized) {
            m->quarantined = hardBad; m->confidence = hardBad ? 0 : target;
        } else if (hardBad) {
            m->goodPending = false;
            if (!m->badPending) { m->badPending = true; m->badSinceUs = s->timestampUs; }
            if ((uint32_t)(s->timestampUs-m->badSinceUs) >= config->rejectUs) m->quarantined = true;
            m->confidence = m->quarantined ? 0 : fminf(m->confidence,0.01f);
        } else {
            m->badPending = false;
            if (m->quarantined) {
                if (!good) m->goodPending = false;
                else if (!m->goodPending) { m->goodPending = true; m->goodSinceUs = s->timestampUs; }
                else if ((uint32_t)(s->timestampUs-m->goodSinceUs) >= config->goodUs) {
                    m->quarantined = false; m->goodPending = false;
                }
            }
            if (m->quarantined) m->confidence = 0;
            else if (target < m->confidence) m->confidence = target;
            else m->confidence += dt/(config->recoverySeconds+dt)*(target-m->confidence);
        }
        if (good && s->earthValid && finiteVector(s->earthField)) {
            if (!m->earthReferenceValid) memcpy(m->earthReference,s->earthField,sizeof(m->earthReference));
            else for (unsigned axis=0;axis<3;axis++) m->earthReference[axis] += 0.05f*(s->earthField[axis]-m->earthReference[axis]);
            m->earthReferenceValid = true;
        }
        m->initialized = true; m->processedAtUs = s->timestampUs;
    }
    float sum = 0, output[3] = {0}; int reference = -1, best = -1;
    for (unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) {
        magFusionMember_t *m=&fusion->member[i];
        if (!m->usable) continue;
        if (samples[i].id == fusion->referenceId) reference = i;
        if (m->confidence > 0) {
            m->contributing = true; fusion->contributors++; sum += m->confidence;
            for (unsigned axis=0;axis<3;axis++) output[axis] += samples[i].field[axis]*m->confidence;
            if (best < 0 || m->confidence > fusion->member[best].confidence ||
                (m->confidence == fusion->member[best].confidence && samples[i].id < samples[best].id)) best=i;
        }
    }
    if (best >= 0 && sum > 0 && isfinite(sum)) {
        if (reference < 0 || !fusion->member[reference].contributing) fusion->referenceId = samples[best].id;
        for (unsigned axis=0;axis<3;axis++) output[axis] /= sum;
        if (finiteVector(output)) { memcpy(fusion->output,output,sizeof(output)); fusion->healthy=true; return true; }
    }
    // All weights are exactly zero: retain the reference's new readable measurement.
    if (reference < 0) for (unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) if (fusion->member[i].usable &&
        (reference < 0 || samples[i].id < samples[reference].id)) reference=i;
    if (reference >= 0) {
        fusion->referenceId = fusion->fallbackId = samples[reference].id;
        fusion->member[reference].contributing = true; fusion->contributors=1;
        memcpy(fusion->output,samples[reference].field,sizeof(fusion->output)); fusion->healthy=true; return true;
    }
    memset(fusion->output,0,sizeof(fusion->output)); fusion->contributors=0;
    return false;
}
