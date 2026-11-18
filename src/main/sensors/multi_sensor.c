/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_SENSOR
#include "sensors/multi_sensor.h"
#include "config/parameter_group_ids.h"
#include "fc/settings.h"

static bool imuMode;
static bool baroMode;
void multiSensorLatchImuMode(void)
{
    imuMode = multiSensorConfig()->imuEnabled;
}

void multiSensorLatchBaroMode(void)
{
    baroMode = multiSensorConfig()->baroEnabled;
}

bool multiSensorImuEnabled(void)
{
    return imuMode;
}

bool multiSensorBaroEnabled(void)
{
    return baroMode;
}

sensorFusion_t multiGyroFusion;
sensorFusion_t multiAccFusion;
sensorFusion_t multiBaroFusion;
sensorFusionSample_t multiGyroSamples[MULTI_IMU_COUNT];
sensorFusionSample_t multiAccSamples[MULTI_IMU_COUNT];
sensorFusionSample_t multiBaroSamples[MULTI_BARO_COUNT];

PG_REGISTER_WITH_RESET_TEMPLATE(multiSensorConfig_t, multiSensorConfig, PG_MULTI_SENSOR_CONFIG, 0);
PG_RESET_TEMPLATE(multiSensorConfig_t, multiSensorConfig,
    .imuEnabled = SETTING_MULTI_IMU_ENABLED_DEFAULT,
    .baroEnabled = SETTING_MULTI_BARO_ENABLED_DEFAULT,
    .gyroDeviationDps = SETTING_MULTI_GYRO_DEVIATION_DEFAULT,
    .accDeviationMg = SETTING_MULTI_ACC_DEVIATION_DEFAULT,
    .baroDeviationPa = SETTING_MULTI_BARO_DEVIATION_DEFAULT,
    .recoveryMs = SETTING_MULTI_SENSOR_RECOVERY_MS_DEFAULT,
    .imuMaxAgeMs = SETTING_MULTI_IMU_MAX_AGE_MS_DEFAULT,
    .baroMaxAgeMs = SETTING_MULTI_BARO_MAX_AGE_MS_DEFAULT,
    .accGain = {{4096,4096,4096},{4096,4096,4096},{4096,4096,4096}},
);

sensorFusionConfig_t multiSensorFusionConfig(float deviation, bool barometer)
{
    const multiSensorConfig_t *config = multiSensorConfig();
    return (sensorFusionConfig_t) {
        .deviation = deviation,
        .recoveryTimeSeconds = config->recoveryMs * 0.001f,
        .maxAgeUs = (uint32_t)(barometer ? config->baroMaxAgeMs : config->imuMaxAgeMs) * 1000,
    };
}
#endif
