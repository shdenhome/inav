/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"
#include "build/debug.h"

#include "common/calibration.h"
#include "common/log.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_bmp388.h"
#include "drivers/barometer/barometer_lps25h.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/barometer/barometer_ms56xx.h"
#include "drivers/barometer/barometer_spl06.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/barometer/barometer_2smpb_02b.h"
#include "drivers/barometer/barometer_msp.h"
#include "drivers/barometer/barometer_crsf.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "sensors/barometer.h"
#include "sensors/multi_sensor.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

baro_t baro;                        // barometer access functions

#ifdef USE_BARO

PG_REGISTER_WITH_RESET_TEMPLATE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 5);

PG_RESET_TEMPLATE(barometerConfig_t, barometerConfig,
    .baro_hardware = SETTING_BARO_HARDWARE_DEFAULT,
    .baro_calibration_tolerance = SETTING_BARO_CAL_TOLERANCE_DEFAULT,
    .baro_temp_correction = SETTING_BARO_TEMP_CORRECTION_DEFAULT,
);

static zeroCalibrationScalar_t zeroCalibration;
static float baroGroundAltitude = 0;
static float baroGroundPressure = 101325.0f; // 101325 pascal, 1 standard atmosphere

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse)
{
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = BARO_NONE;
    requestedSensors[SENSOR_INDEX_BARO] = baroHardwareToUse;

    switch (baroHardwareToUse) {
    case BARO_AUTODETECT:
    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        if (bmp085Detect(dev)) {
            baroHardware = BARO_BMP085;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_MS5607:
#ifdef USE_BARO_MS5607
        if (ms5607Detect(dev)) {
            baroHardware = BARO_MS5607;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_MS5611:
#ifdef USE_BARO_MS5611
        if (ms5611Detect(dev)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(dev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_BMP388:
#if defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388)
        if (bmp388Detect(dev)) {
            baroHardware = BARO_BMP388;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_SPL06:
#if defined(USE_BARO_SPL06) || defined(USE_BARO_SPI_SPL06)
        if (spl06Detect(dev)) {
            baroHardware = BARO_SPL06;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_LPS25H:
#if defined(USE_BARO_LPS25H)
        if (lps25hDetect(dev)) {
            baroHardware = BARO_LPS25H;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_DPS310:
#if defined(USE_BARO_DPS310)
        if (baroDPS310Detect(dev)) {
            baroHardware = BARO_DPS310;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_B2SMPB:
#if defined(USE_BARO_B2SMPB)
        if (baro2SMPB02BDetect(dev)) {
            baroHardware = BARO_B2SMPB;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_MSP:
#ifdef USE_BARO_MSP
        // Skip autodetection for MSP baro, only allow manual config
        if (baroHardwareToUse != BARO_AUTODETECT && mspBaroDetect(dev)) {
            baroHardware = BARO_MSP;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_CRSF:
#ifdef USE_BARO_CRSF
        // Skip autodetection for CRSF baro, only allow manual config
        if (baroHardwareToUse != BARO_AUTODETECT && crsfBaroDetect(dev)) {
            baroHardware = BARO_CRSF;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_FAKE:
#ifdef USE_FAKE_BARO
        if (fakeBaroDetect(dev)) {
            baroHardware = BARO_FAKE;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }
        FALLTHROUGH;

    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    if (baroHardware == BARO_NONE) {
        sensorsClear(SENSOR_BARO);
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}

#ifdef USE_MULTI_SENSOR
typedef struct multiBarometer_s {
    baroDev_t dev;
    uint32_t dueUs;
    int32_t temperature;
    bool detected;
    bool pressurePending;
    bool conversionStarted;
} multiBarometer_t;
static multiBarometer_t multiBarometer[MULTI_BARO_COUNT];
static bool multiBaroNewSample;
static zeroCalibrationScalar_t multiBaroDifferenceCalibration;
static bool multiBaroAlignmentActive;
static bool multiBaroAlignmentReady;
static float multiBaroDifference;

float baroGetPressureOffset(unsigned slot)
{
    if (!multiBaroAlignmentActive || !multiBaroAlignmentReady || slot >= MULTI_BARO_COUNT) {
        return 0;
    }
    return (slot == 0 ? 0.5f : -0.5f) * multiBaroDifference;
}


static bool multiBaroInit(void)
{
    memset(multiBarometer, 0, sizeof(multiBarometer));
    memset(multiBaroSamples, 0, sizeof(multiBaroSamples));
    sensorFusionReset(&multiBaroFusion);
    multiBaroNewSample = false;
    multiBaroAlignmentActive = false;
    multiBaroAlignmentReady = false;
    multiBaroDifference = 0;
    // These are different chips with independent driver state, on distinct addresses.
    multiBarometer[0].detected = ms5611Detect(&multiBarometer[0].dev);
    multiBarometer[1].detected = spl06Detect(&multiBarometer[1].dev);
    const bool present = multiBarometer[0].detected || multiBarometer[1].detected;
    if (present) {
        detectedSensors[SENSOR_INDEX_BARO] = multiBarometer[0].detected ? BARO_MS5611 : BARO_SPL06;
        sensorsSet(SENSOR_BARO);
    } else {
        detectedSensors[SENSOR_INDEX_BARO] = BARO_NONE;
        sensorsClear(SENSOR_BARO);
    }
    requestedSensors[SENSOR_INDEX_BARO] = BARO_AUTODETECT;
    const uint32_t now = micros();
    for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
        multiBarometer_t *sensor = &multiBarometer[i];
        if (sensor->detected) {
            sensor->conversionStarted = sensor->dev.start_ut(&sensor->dev);
            sensor->dueUs = now + sensor->dev.ut_delay;
        }
    }
    return present;
}

static uint32_t multiBaroUpdate(void)
{
    const uint32_t now = micros();
    uint32_t next = 10000;
    multiBaroNewSample = false;
    bool newPressure[MULTI_BARO_COUNT] = {false};
    for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
        multiBarometer_t *sensor = &multiBarometer[i];
        if (!sensor->detected) {
            continue;
        }
        if ((int32_t)(now - sensor->dueUs) >= 0) {
            bool ok = sensor->conversionStarted;
            if (sensor->pressurePending) {
                ok = ok && sensor->dev.get_up(&sensor->dev);
                int32_t pressure = 0, temperature = 0;
                ok = ok && sensor->dev.calculate(&sensor->dev, &pressure, &temperature);
                ok = ok && pressure >= 1000 && pressure <= 150000 && temperature >= -10000 && temperature <= 15000;
                multiBaroSamples[i].valid = ok;
                if (ok) {
                    multiBaroSamples[i].value[0] = pressure;
                    multiBaroSamples[i].timestampUs = now;
                    sensor->temperature = temperature;
                    newPressure[i] = true;
                }
                sensor->pressurePending = false;
                sensor->conversionStarted = sensor->dev.start_ut(&sensor->dev);
                sensor->dueUs = now + sensor->dev.ut_delay;
            } else {
                ok = ok && sensor->dev.get_ut(&sensor->dev);
                if (ok) {
                    sensor->conversionStarted = sensor->dev.start_up(&sensor->dev);
                    sensor->pressurePending = true;
                    sensor->dueUs = now + sensor->dev.up_delay;
                } else {
                    multiBaroSamples[i].valid = false;
                    sensor->conversionStarted = sensor->dev.start_ut(&sensor->dev);
                    sensor->dueUs = now + sensor->dev.ut_delay;
                }
            }
            if (!sensor->conversionStarted) {
                multiBaroSamples[i].valid = false;
                // Retry temperature acquisition on the next service, never read a failed conversion.
                sensor->pressurePending = false;
                sensor->dueUs = now + 10000;
            }
        }
        const int32_t wait = (int32_t)(sensor->dueUs - now);
        const uint32_t remaining = wait > 0 ? (uint32_t)wait : 1000;
        if (remaining < next) {
            next = remaining;
        }
    }
    const sensorFusionConfig_t config = multiSensorFusionConfig(multiSensorConfig()->baroDeviationPa, true);
    if (multiBaroAlignmentActive && !multiBaroAlignmentReady) {
        bool pairFresh = true;
        for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
            pairFresh = pairFresh && multiBaroSamples[i].valid &&
                (uint32_t)(now - multiBaroSamples[i].timestampUs) <= config.maxAgeUs;
        }
        if (!pairFresh) {
            zeroCalibrationStartS(&multiBaroDifferenceCalibration, CALIBRATING_BARO_TIME_MS,
                multiBaroDifferenceCalibration.params.stdDevThreshold, false);
        } else if (newPressure[0] || newPressure[1]) {
            zeroCalibrationAddValueS(&multiBaroDifferenceCalibration,
                multiBaroSamples[1].value[0] - multiBaroSamples[0].value[0]);
            if (zeroCalibrationIsCompleteS(&multiBaroDifferenceCalibration) &&
                zeroCalibrationIsSuccessfulS(&multiBaroDifferenceCalibration)) {
                float difference;
                zeroCalibrationGetZeroS(&multiBaroDifferenceCalibration, &difference);
                // A gross discrepancy is not a ground-datum adjustment. Do not hide
                // an implausible source behind a very large learned offset.
                if (fabsf(difference) > multiSensorConfig()->baroDeviationPa * 10.0f) {
                    zeroCalibrationStartS(&multiBaroDifferenceCalibration, CALIBRATING_BARO_TIME_MS,
                        multiBaroDifferenceCalibration.params.stdDevThreshold, false);
                } else {
                    multiBaroDifference = difference;
                    multiBaroAlignmentReady = true;
                    sensorFusionReset(&multiBaroFusion);
                    // Re-establish the common ground reference in the aligned datum.
                    zeroCalibrationStartS(&zeroCalibration, CALIBRATING_BARO_TIME_MS,
                        zeroCalibration.params.stdDevThreshold, false);
                }
            }
        }
    }
    sensorFusionSample_t aligned[MULTI_BARO_COUNT];
    memcpy(aligned, multiBaroSamples, sizeof(aligned));
    for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
        aligned[i].value[0] += baroGetPressureOffset(i);
        aligned[i].valid = aligned[i].valid && aligned[i].value[0] >= 1000 && aligned[i].value[0] <= 150000;
    }
    if (sensorFusionUpdate(&multiBaroFusion, aligned, MULTI_BARO_COUNT, 1, now, &config)) {
        baro.baroPressure = lrintf(multiBaroFusion.output[0]);
        float temperature = 0, sum = 0;
        for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
            if (multiBaroFusion.member[i].usable && multiBaroFusion.member[i].confidence >= SENSOR_FUSION_MIN_CONFIDENCE) {
                multiBaroNewSample = multiBaroNewSample || newPressure[i];
                temperature += multiBarometer[i].temperature * multiBaroFusion.member[i].confidence;
                sum += multiBaroFusion.member[i].confidence;
            }
        }
        if (sum > 0) {
            baro.baroTemperature = lrintf(temperature / sum);
        }
    } else {
        multiBaroNewSample = false;
    }
    return next < 1000 ? 1000 : next;
}
#endif

bool baroHasNewSample(void)
{
#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled()) {
        return multiBaroNewSample && multiBaroFusion.healthy;
    }
#endif
    return true;
}

bool baroInit(void)
{
#ifdef USE_MULTI_SENSOR
    multiSensorLatchBaroMode();
    if (multiSensorBaroEnabled()) {
        return multiBaroInit();
    }
#endif

    if (!baroDetect(&baro.dev, barometerConfig()->baro_hardware)) {
        return false;
    }
    return true;
}

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION
} barometerState_e;

uint32_t baroUpdate(void)
{
    static barometerState_e state = BAROMETER_NEEDS_SAMPLES;

#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE_HITL)) {
        return 0;
    }
#endif

#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled()) {
        return multiBaroUpdate();
    }
#endif

    switch (state) {
        default:
        case BAROMETER_NEEDS_SAMPLES:
            if (baro.dev.get_ut) {
                baro.dev.get_ut(&baro.dev);
            }
            if (baro.dev.start_up) {
                baro.dev.start_up(&baro.dev);
            }
            state = BAROMETER_NEEDS_CALCULATION;
            return baro.dev.up_delay;
        break;

        case BAROMETER_NEEDS_CALCULATION:
            if (baro.dev.get_up) {
                baro.dev.get_up(&baro.dev);
            }
            if (baro.dev.start_ut) {
                baro.dev.start_ut(&baro.dev);
            }
            //output: baro.baroPressure, baro.baroTemperature
            baro.dev.calculate(&baro.dev, &baro.baroPressure, &baro.baroTemperature);
            state = BAROMETER_NEEDS_SAMPLES;
            return baro.dev.ut_delay;
        break;
    }
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

float altitudeToPressure(const float altCm)
{
    return powf(1.0f - (altCm / 4433000.0f), 5.254999) * 101325.0f;
}

bool baroIsCalibrationComplete(void)
{
#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled() && multiBaroAlignmentActive && !multiBaroAlignmentReady) {
        return false;
    }
#endif
    return zeroCalibrationIsCompleteS(&zeroCalibration) && zeroCalibrationIsSuccessfulS(&zeroCalibration);
}

void baroStartCalibration(void)
{
    const float acceptedPressureVariance = (101325.0f - altitudeToPressure(barometerConfig()->baro_calibration_tolerance)); // max 30cm deviation during calibration (at sea level)
    zeroCalibrationStartS(&zeroCalibration, CALIBRATING_BARO_TIME_MS, acceptedPressureVariance, false);
#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled()) {
        multiBaroAlignmentActive = true;
        multiBaroAlignmentReady = !(multiBarometer[0].detected && multiBarometer[1].detected);
        multiBaroDifference = 0;
        zeroCalibrationStartS(&multiBaroDifferenceCalibration, CALIBRATING_BARO_TIME_MS,
            fmaxf(1, acceptedPressureVariance * 2), false);
        sensorFusionReset(&multiBaroFusion);
    }
#endif
}

int32_t baroCalculateAltitude(void)
{
#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled() && !multiBaroFusion.healthy) {
        return baro.BaroAlt;
    }
#endif
    if (!baroIsCalibrationComplete()) {
        zeroCalibrationAddValueS(&zeroCalibration, baro.baroPressure);

        if (zeroCalibrationIsCompleteS(&zeroCalibration)) {
            zeroCalibrationGetZeroS(&zeroCalibration, &baroGroundPressure);
            baroGroundAltitude = pressureToAltitude(baroGroundPressure);
            LOG_DEBUG(BARO, "Barometer calibration complete (%d)", (int)lrintf(baroGroundAltitude));
        }

        baro.BaroAlt = 0;
    }
    else {
        // calculates height from ground via baro readings
        baro.BaroAlt = pressureToAltitude(baro.baroPressure) - baroGroundAltitude;
#ifdef USE_MULTI_SENSOR
        // The legacy correction belongs to a single sensor. Both drivers already
        // compensate their own pressure; never apply that single-sensor fit to a blend.
        if (!multiSensorBaroEnabled())
#endif
        baro.BaroAlt += applySensorTempCompensation(baro.baroTemperature, baro.BaroAlt, SENSOR_INDEX_BARO);
   }

    return baro.BaroAlt;
}

int32_t baroGetLatestAltitude(void)
{
    return baro.BaroAlt;
}

int16_t baroGetTemperature(void)
{
    return CENTIDEGREES_TO_DECIDEGREES(baro.baroTemperature);
}

bool baroIsHealthy(void)
{
#ifdef USE_MULTI_SENSOR
    if (multiSensorBaroEnabled()) {
        return multiBaroFusion.healthy;
    }
#endif
    return sensors(SENSOR_BARO);
}

#endif /* BARO */
