/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "sensors/multi_sensor.h"
#include "sensors/multi_sensor_msp.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"
}
#include "gtest/gtest.h"
namespace {
uint8_t accHardware[3], baroHardware[2], primaryTag;
}
extern "C" {
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
uint8_t gyroGetImuTag(unsigned slot) { return slot == 0 && multiSensorImuEnabled() ? primaryTag : UINT8_MAX; }
uint8_t multiAccGetHardware(unsigned tag) { return tag < 3 ? accHardware[tag] : 0; }
uint8_t baroGetSensorHardware(unsigned slot) { return multiSensorBaroEnabled() && slot < 2 ? baroHardware[slot] : 0; }
}
class MultiSensorMspTest : public ::testing::Test {
protected:
    uint8_t bytes[12];
    void SetUp() override {
        *multiSensorConfigMutable() = pgResetTemplate_multiSensorConfig;
        primaryTag = 0;
        accHardware[0] = accHardware[2] = ACC_ICM42605;
        accHardware[1] = ACC_MPU6500;
        baroHardware[0] = BARO_MS5611; baroHardware[1] = BARO_SPL06;
        detectedSensors[SENSOR_INDEX_BARO] = BARO_MS5611;
        memset(bytes, 0xa5, sizeof(bytes));
    }
    void serialize(bool imu, bool baro) {
        multiSensorConfigMutable()->imuEnabled = imu;
        multiSensorConfigMutable()->baroEnabled = baro;
        multiSensorLatchImuMode(); multiSensorLatchBaroMode();
        sbuf_t dst; sbufInit(&dst, bytes, bytes + 11);
        multiSensorSerializeInfo(&dst);
        EXPECT_EQ(sbufPtr(&dst), bytes + 11);
        EXPECT_EQ(bytes[11], 0xa5);
    }
};
TEST_F(MultiSensorMspTest, WireLayoutPreservesDuplicateTypesAndPhysicalTag) {
    primaryTag = 2; serialize(true, true);
    const uint8_t expected[] = {'M','S',1,3,2,ACC_ICM42605,ACC_MPU6500,ACC_ICM42605,BARO_MS5611,BARO_SPL06,BARO_MS5611};
    EXPECT_EQ(memcmp(bytes, expected, sizeof(expected)), 0);
}
TEST_F(MultiSensorMspTest, DisabledModesDoNotReportInactiveAdditionalDevices) {
    serialize(false, false);
    EXPECT_EQ(bytes[3], 0); EXPECT_EQ(bytes[4], 255);
    for (unsigned i = 5; i < 10; i++) EXPECT_EQ(bytes[i], 0);
}
TEST_F(MultiSensorMspTest, MissingPhysicalDeviceDoesNotShiftOtherTagsOrClaimDetection) {
    accHardware[1] = 0; baroHardware[0] = 0;
    detectedSensors[SENSOR_INDEX_BARO] = BARO_SPL06;
    serialize(true, true);
    EXPECT_EQ(bytes[6], 0); EXPECT_EQ(bytes[7], ACC_ICM42605);
    EXPECT_EQ(bytes[8], 0); EXPECT_EQ(bytes[9], BARO_SPL06);
    EXPECT_EQ(bytes[10], BARO_SPL06);
}
TEST_F(MultiSensorMspTest, ModesAreIndependent) {
    serialize(true, false); EXPECT_EQ(bytes[3], 1);
    serialize(false, true); EXPECT_EQ(bytes[3], 2);
}
