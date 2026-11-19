/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "drivers/time.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/multi_sensor.h"
#include "io/osd_sensor_trust.h"
}
#include "gtest/gtest.h"
namespace {
uint32_t clockUs;
uint8_t tags[3];
bool gyroCalibrated, accCalibrated;
}
extern "C" {
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
timeUs_t micros(void) { return clockUs; }
timeMs_t millis(void) { return clockUs / 1000; }
uint8_t gyroGetImuTag(unsigned slot) { return slot < 3 ? tags[slot] : UINT8_MAX; }
bool gyroIsCalibrationComplete(void) { return gyroCalibrated; }
bool accIsCalibrationComplete(void) { return accCalibrated; }
}
class OsdSensorTrustTest : public ::testing::Test {
protected:
    void refresh() {
        for (unsigned i = 0; i < 3; i++) {
            multiGyroSamples[i].timestampUs = multiAccSamples[i].timestampUs = clockUs;
        }
        for (unsigned i = 0; i < 2; i++) multiBaroSamples[i].timestampUs = clockUs;
    }
    void tick(unsigned count = 1) {
        for (unsigned i = 0; i < count; i++) { clockUs += 100000; refresh(); osdSensorTrustUpdate(); }
    }
    void SetUp() override {
        *multiSensorConfigMutable() = pgResetTemplate_multiSensorConfig;
        multiSensorLatchImuMode(); multiSensorLatchBaroMode();
        clockUs += 200000; osdSensorTrustUpdate();
        memset(&multiGyroFusion, 0, sizeof(multiGyroFusion));
        memset(&multiAccFusion, 0, sizeof(multiAccFusion));
        memset(&multiBaroFusion, 0, sizeof(multiBaroFusion));
        gyroCalibrated = accCalibrated = true;
        for (unsigned i = 0; i < 3; i++) {
            tags[i] = i;
            multiGyroFusion.member[i].initialized = multiAccFusion.member[i].initialized = true;
            multiGyroFusion.member[i].usable = multiAccFusion.member[i].usable = true;
            multiGyroFusion.member[i].confidence = multiAccFusion.member[i].confidence = 1;
            multiGyroSamples[i].valid = multiAccSamples[i].valid = true;
        }
        for (unsigned i = 0; i < 2; i++) {
            multiBaroFusion.member[i].initialized = multiBaroFusion.member[i].usable = true;
            multiBaroFusion.member[i].confidence = 1;
            multiBaroSamples[i].valid = true;
        }
        multiSensorConfigMutable()->imuEnabled = multiSensorConfigMutable()->baroEnabled = true;
        multiSensorConfigMutable()->accCalibratedMask = 7;
        multiSensorLatchImuMode(); multiSensorLatchBaroMode();
        tick();
    }
};
TEST_F(OsdSensorTrustTest, StartupDoesNotAnnounceRestoration) {
    EXPECT_EQ(osdSensorTrustMessage(), nullptr);
}
TEST_F(OsdSensorTrustTest, ImuLossAndRestorationUsePhysicalTagNotRuntimeSlot) {
    multiSensorConfigMutable()->imuEnabled = false; multiSensorLatchImuMode(); tick();
    tags[0] = 2; tags[1] = 0; tags[2] = 1;
    multiSensorConfigMutable()->imuEnabled = true; multiSensorLatchImuMode(); tick();
    multiGyroFusion.member[0].confidence = 0; tick(11);
    ASSERT_NE(osdSensorTrustMessage(), nullptr);
    EXPECT_STREQ(osdSensorTrustMessage(), "IMU 3 TRUST LOST");
    multiGyroFusion.member[0].confidence = 1; tick(11);
    for (unsigned i = 0; i < 7; i++) {
        osdSensorTrustSetVisible(true); clockUs += 500000; osdSensorTrustMessage();
    }
    EXPECT_STREQ(osdSensorTrustMessage(), "IMU 3 TRUST RESTORED");
}
TEST_F(OsdSensorTrustTest, CalibratedAccelerometerFailureAlsoWarnsForItsImu) {
    multiAccFusion.member[1].usable = false; tick(11);
    EXPECT_STREQ(osdSensorTrustMessage(), "IMU 2 TRUST LOST");
}
TEST_F(OsdSensorTrustTest, UncalibratedAccelerometersDoNotGenerateSpuriousLoss) {
    multiSensorConfigMutable()->accCalibratedMask = 0;
    multiAccFusion.member[1].usable = false; tick(11);
    EXPECT_EQ(osdSensorTrustMessage(), nullptr);
}
TEST_F(OsdSensorTrustTest, BarometersUseTheirOwnNamesAndIndependentEvents) {
    multiBaroSamples[0].valid = false;
    multiBaroFusion.member[1].confidence = 0.1f; tick(11);
    EXPECT_STREQ(osdSensorTrustMessage(), "MS5611 TRUST LOST");
    for (unsigned i = 0; i < 6; i++) {
        osdSensorTrustSetVisible(true); clockUs += 500000; osdSensorTrustMessage();
    }
    EXPECT_STREQ(osdSensorTrustMessage(), "SPL06 TRUST LOST");
}
TEST_F(OsdSensorTrustTest, StaleSamplesCauseLossEvenIfFusionSnapshotWasPreviouslyHealthy) {
    for (unsigned i = 0; i < 12; i++) { clockUs += 100000; osdSensorTrustUpdate(); }
    EXPECT_STREQ(osdSensorTrustMessage(), "IMU 1 TRUST LOST");
}
TEST_F(OsdSensorTrustTest, IntentionalCalibrationSuppressesImuTrustEvents) {
    accCalibrated = false;
    multiGyroFusion.member[0].confidence = 0; tick(12);
    EXPECT_EQ(osdSensorTrustMessage(), nullptr);
    accCalibrated = true; multiGyroFusion.member[0].confidence = 1; tick(12);
    EXPECT_EQ(osdSensorTrustMessage(), nullptr);
}
