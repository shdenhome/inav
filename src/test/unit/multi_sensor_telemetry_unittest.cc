/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
#include <limits>
#include <fstream>
#include <cstdlib>
extern "C" {
#include "platform.h"
#include "sensors/multi_sensor_telemetry.h"
#include "sensors/multi_acc.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
}
#include "gtest/gtest.h"
namespace {
uint8_t tags[3], hardware[3], baroHardware[2];
bool rawRead[3], heightValid[2], rotate;
uint32_t now;
}
extern "C" {
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
uint8_t gyroGetImuTag(unsigned slot) { return slot < 3 ? tags[slot] : UINT8_MAX; }
uint8_t multiAccGetHardware(unsigned tag) { return tag < 3 ? hardware[tag] : 0; }
bool multiAccGetCalibrationStatus(unsigned slot, multiAccCalibrationStatus_t *status) {
    memset(status,0,sizeof(*status)); status->detected = tags[slot] < 3;
    status->readValid = rawRead[slot]; status->uncalibratedG[2] = 1.25f;
    return true;
}
uint8_t baroGetSensorHardware(unsigned slot) { return slot < 2 ? baroHardware[slot] : 0; }
float baroGetPressureOffset(unsigned slot) { return slot == 0 ? -70 : 70; }
int32_t baroGetSensorTemperature(unsigned slot) { return slot == 0 ? -1200 : 2534; }
bool baroGetSensorAltitude(unsigned slot, int32_t *altitude) { *altitude = slot == 0 ? -123 : 456; return heightValid[slot]; }
void applyBoardAlignment(float *vector) { if (rotate) { float x=vector[0]; vector[0]=-vector[1]; vector[1]=x; } }
}
class MultiSensorTelemetryTest : public ::testing::Test {
protected:
    multiSensorTelemetry_t state;
    void SetUp() override {
        now=100000; rotate=false;
        *multiSensorConfigMutable()=pgResetTemplate_multiSensorConfig;
        multiSensorConfigMutable()->imuEnabled=multiSensorConfigMutable()->baroEnabled=true;
        multiSensorConfigMutable()->accCalibratedMask=7;
        multiSensorLatchImuMode(); multiSensorLatchBaroMode();
        memset(&multiGyroFusion,0,sizeof(multiGyroFusion)); memset(&multiAccFusion,0,sizeof(multiAccFusion));
        memset(&multiBaroFusion,0,sizeof(multiBaroFusion));
        multiGyroFusion.contributorMask = multiAccFusion.contributorMask = 7;
        for(unsigned i=0;i<3;i++) {
            tags[i]=i; hardware[i]=i==1 ? 3 : 8; rawRead[i]=true;
            auto &g=multiGyroFusion.member[i]; auto &a=multiAccFusion.member[i];
            g.initialized=a.initialized=g.usable=a.usable=true; g.confidence=a.confidence=0.9f;
            multiGyroSamples[i]={{1.25f,-2.5f,3.75f},now-200,true};
            multiAccSamples[i]={{0.125f,-0.25f,1.0f},now-400,true};
        }
        for(unsigned i=0;i<2;i++) {
            baroHardware[i]=i==0 ? 3 : 7; heightValid[i]=true;
            auto &m=multiBaroFusion.member[i];m.initialized=m.usable=true;m.confidence=0.8f;
            multiBaroSamples[i]={{100000.0f+i*10,0,0},now-10000,true};
        }
    }
    void capture() { multiSensorCaptureTelemetry(&state,now); }
};
TEST_F(MultiSensorTelemetryTest, CapturesPhysicalUnitsTrustAndBarometerReference) {
    capture(); EXPECT_EQ(state.flags,3);EXPECT_EQ(state.timestampUs,now);
    const auto &imu=state.imu[0];
    EXPECT_EQ(imu.flags,127);EXPECT_EQ(imu.gyroMdps[0],1250);EXPECT_EQ(imu.gyroMdps[1],-2500);
    EXPECT_EQ(imu.accMg[0],125);EXPECT_EQ(imu.rawAccMg[2],1250);
    EXPECT_EQ(imu.gyroAgeUs,200u);EXPECT_EQ(imu.accAgeUs,400u);EXPECT_EQ(imu.accTrust,900);
    EXPECT_EQ(state.baro[0].pressurePa,100000);EXPECT_EQ(state.baro[0].alignedPressurePa,99930);
    EXPECT_EQ(state.baro[0].temperatureCenti,-1200);EXPECT_EQ(state.baro[0].altitudeCm,-123);
    EXPECT_EQ(state.baro[1].altitudeCm,456);EXPECT_EQ(state.baro[1].flags,15);
}
TEST_F(MultiSensorTelemetryTest, RuntimeSlotOrderDoesNotRenumberPhysicalTags) {
    tags[0]=2;tags[1]=0;tags[2]=1; multiGyroSamples[0].value[0]=12;
    capture(); EXPECT_EQ(state.imu[2].gyroMdps[0],12000);EXPECT_EQ(state.imu[2].tag,2);
    EXPECT_EQ(state.imu[1].hardware,3);
}
TEST_F(MultiSensorTelemetryTest, StaleAndQuarantinedMeasurementsAreDistinguished) {
    multiGyroSamples[0].timestampUs=now-20000;
    multiAccFusion.member[1].confidence=0.01f;multiAccFusion.contributorMask &= ~(1u << 1);capture();
    EXPECT_FALSE(state.imu[0].flags & MULTI_IMU_GYRO_VALID);
    EXPECT_TRUE(state.imu[1].flags & MULTI_IMU_ACC_VALID);
    EXPECT_FALSE(state.imu[1].flags & MULTI_IMU_ACC_CONTRIBUTOR);
    EXPECT_EQ(state.imu[1].accTrust,10);
}
TEST_F(MultiSensorTelemetryTest, UncalibratedRawAccelerationRemainsAvailableAndAlignedOnce) {
    rotate=true;multiSensorConfigMutable()->accCalibratedMask=0;capture();
    EXPECT_FALSE(state.imu[0].flags & MULTI_IMU_ACC_VALID);
    EXPECT_TRUE(state.imu[0].flags & MULTI_IMU_RAW_ACC_VALID);
    EXPECT_EQ(state.imu[0].accMg[0],250);EXPECT_EQ(state.imu[0].accMg[1],125);
    EXPECT_EQ(state.imu[0].gyroMdps[0],1250);
}
TEST_F(MultiSensorTelemetryTest, MissingAndDisabledSourcesAreNotInvented) {
    tags[1]=UINT8_MAX;baroHardware[0]=0;capture();
    EXPECT_EQ(state.imu[1].flags,0);EXPECT_EQ(state.imu[1].gyroAgeUs,UINT32_MAX);
    EXPECT_EQ(state.baro[0].flags,0);
    multiSensorConfigMutable()->imuEnabled=multiSensorConfigMutable()->baroEnabled=false;
    multiSensorLatchImuMode();multiSensorLatchBaroMode();capture();
    EXPECT_EQ(state.flags,0);EXPECT_EQ(state.imu[0].flags,0);EXPECT_EQ(state.baro[1].flags,0);
}
TEST_F(MultiSensorTelemetryTest, NonfiniteAndExtremeValuesCannotOverflowIntegerConversion) {
    multiGyroSamples[0].value[0]=std::numeric_limits<float>::quiet_NaN();
    multiGyroSamples[1].value[0]=std::numeric_limits<float>::max();
    multiAccFusion.member[0].confidence=std::numeric_limits<float>::infinity();capture();
    EXPECT_FALSE(state.imu[0].flags & MULTI_IMU_GYRO_VALID);EXPECT_EQ(state.imu[0].gyroMdps[0],0);
    EXPECT_EQ(state.imu[1].gyroMdps[0],0); // scaled infinity is invalid, not an overflowing conversion
    EXPECT_EQ(state.imu[0].accTrust,0);
}
TEST_F(MultiSensorTelemetryTest, TimeWrapUsesUnsignedSampleAge) {
    now=100;multiGyroSamples[0].timestampUs=UINT32_MAX-99;capture();
    EXPECT_EQ(state.imu[0].gyroAgeUs,200u);EXPECT_TRUE(state.imu[0].flags & MULTI_IMU_GYRO_VALID);
}
TEST_F(MultiSensorTelemetryTest, SerializesExactly211BytesWithSignedLittleEndianValues) {
    capture();uint8_t bytes[MULTI_SENSOR_STATE_BYTES+1];memset(bytes,0xa5,sizeof(bytes));
    sbuf_t dst;sbufInit(&dst,bytes,bytes+MULTI_SENSOR_STATE_BYTES);multiSensorSerializeTelemetry(&dst,&state);
    EXPECT_EQ(sbufPtr(&dst),bytes+MULTI_SENSOR_STATE_BYTES);EXPECT_EQ(bytes[MULTI_SENSOR_STATE_BYTES],0xa5);
    sbuf_t src;sbufInit(&src,bytes,bytes+MULTI_SENSOR_STATE_BYTES);
    EXPECT_EQ(sbufReadU8(&src),1);EXPECT_EQ(sbufReadU8(&src),3);EXPECT_EQ(sbufReadU8(&src),3);EXPECT_EQ(sbufReadU8(&src),2);
    EXPECT_EQ(sbufReadU32(&src),now);EXPECT_EQ(sbufReadU8(&src),0);EXPECT_EQ(sbufReadU8(&src),8);
    EXPECT_EQ(sbufReadU8(&src),127);EXPECT_EQ(sbufReadU16(&src),900);EXPECT_EQ(sbufReadU16(&src),900);
    EXPECT_EQ(sbufReadU32(&src),200u);EXPECT_EQ(sbufReadU32(&src),400u);
    EXPECT_EQ((int32_t)sbufReadU32(&src),1250);EXPECT_EQ((int32_t)sbufReadU32(&src),-2500);
    EXPECT_EQ((int32_t)(uint32_t)(bytes[207]|bytes[208]<<8|bytes[209]<<16|bytes[210]<<24),456);
    if(const char *path=std::getenv("INAV_TEST_TELEMETRY_FIXTURE")) {
        std::ofstream file(path,std::ios::binary);ASSERT_TRUE(file.is_open());
        file.write(reinterpret_cast<const char*>(bytes),MULTI_SENSOR_STATE_BYTES);ASSERT_TRUE(file.good());
    }
}

TEST_F(MultiSensorTelemetryTest, LowTrustFallbackRemainsAContributorWithoutInflatingConfidence) {
    multiGyroFusion.member[1].confidence = 0;
    multiAccFusion.member[1].confidence = 0.02f;
    multiGyroFusion.contributorMask = multiAccFusion.contributorMask = 1u << 1;
    capture();
    EXPECT_TRUE(state.imu[1].flags & MULTI_IMU_GYRO_CONTRIBUTOR);
    EXPECT_TRUE(state.imu[1].flags & MULTI_IMU_ACC_CONTRIBUTOR);
    EXPECT_EQ(state.imu[1].gyroTrust, 0); EXPECT_EQ(state.imu[1].accTrust, 20);
    EXPECT_FALSE(state.imu[0].flags & MULTI_IMU_GYRO_CONTRIBUTOR);
    EXPECT_FALSE(state.imu[2].flags & MULTI_IMU_ACC_CONTRIBUTOR);
}
