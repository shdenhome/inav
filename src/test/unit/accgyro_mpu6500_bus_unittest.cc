/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "common/log.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
}
#include "gtest/gtest.h"
namespace {
busDevice_t bus;
uint32_t scratch[(BUS_SCRATCHPAD_MEMORY_SIZE+3)/4];
uint8_t registers[256];
int failedRead,failedWrite;
bool poweredWithInterrupts;
unsigned released;
}
extern "C" {
busDevice_t *busDeviceInit(busType_e, devHardwareType_e, uint8_t, resourceOwner_e) { return &bus; }
busDevice_t *busDeviceOpen(busType_e, devHardwareType_e, uint8_t) { return &bus; }
void busDeviceDeInit(busDevice_t *) { released++; }
void *busDeviceGetScratchpadMemory(const busDevice_t *) { return scratch; }
void busSetSpeed(const busDevice_t *, busSpeed_e) {}
void delay(timeMs_t) {}
void _logf(logTopic_e, unsigned, const char *, ...) {}
bool busRead(const busDevice_t *, uint8_t reg, uint8_t *data) {
    if (reg==failedRead) return false;
    *data=registers[reg]; return true;
}
bool busReadBuf(const busDevice_t *, uint8_t reg, uint8_t *data, uint8_t length) {
    if (reg==failedRead) return false;
    memcpy(data,registers+reg,length); return true;
}
bool busWrite(const busDevice_t *, uint8_t reg, uint8_t data) {
    if (reg==failedWrite) return false;
    if (reg==MPU_RA_PWR_MGMT_1 && data==0 && registers[MPU_RA_INT_ENABLE]) poweredWithInterrupts=true;
    registers[reg]=data; return true;
}
}
class Mpu6500BusTest : public ::testing::Test {
protected:
    gyroDev_t gyro{};
    accDev_t acc{};
    void SetUp() override {
        memset(registers,0,sizeof(registers)); memset(scratch,0,sizeof(scratch));
        memset(&bus,0,sizeof(bus));
        failedRead=failedWrite=-1; released=0; poweredWithInterrupts=false;
        registers[MPU_RA_WHO_AM_I]=ICM20602_WHO_AM_I_CONST;
        gyro.requestedSampleIntervalUs=1000; gyro.lpf=GYRO_LPF_256HZ;
    }
};
TEST_F(Mpu6500BusTest, ExistingIcm20602IdentityRangesAndSampleRateArePreserved) {
    ASSERT_TRUE(mpu6500GyroDetect(&gyro)); gyro.initFn(&gyro);
    EXPECT_EQ(gyro.sampleRateIntervalUs,1000u);
    EXPECT_FLOAT_EQ(gyro.scale,1.0f/16.4f);
    EXPECT_EQ(registers[MPU_RA_GYRO_CONFIG],3<<3);
    EXPECT_EQ(registers[MPU_RA_ACCEL_CONFIG],3<<3);
    ASSERT_TRUE(mpu6500AccDetect(&acc)); acc.initFn(&acc);
    EXPECT_EQ(acc.acc_1G,2048);
}
TEST_F(Mpu6500BusTest, PollingModeClearsSharedInterruptMappingBeforeWake) {
    registers[MPU_RA_INT_ENABLE]=1;
    ASSERT_TRUE(mpu6500GyroDetect(&gyro));
    gyro.pollingOnly=true; gyro.initFn(&gyro);
    EXPECT_EQ(registers[MPU_RA_INT_ENABLE],0);
    EXPECT_FALSE(poweredWithInterrupts);
    EXPECT_NE(gyro.sampleRateIntervalUs,0u);
}
TEST_F(Mpu6500BusTest, FailedInterruptDisableAbortsPollingInitialization) {
    ASSERT_TRUE(mpu6500GyroDetect(&gyro));
    gyro.pollingOnly=true; failedWrite=MPU_RA_INT_ENABLE;
    gyro.initFn(&gyro);
    EXPECT_EQ(gyro.sampleRateIntervalUs,0u);
}
TEST_F(Mpu6500BusTest, IdentityNackCannotDetectAChipFromAnUninitializedByte) {
    failedRead=MPU_RA_WHO_AM_I;
    EXPECT_FALSE(mpu6500GyroDetect(&gyro)); EXPECT_EQ(released,1u);
}
TEST_F(Mpu6500BusTest, CachedAccelerometerReadTracksGyroTransferSuccess) {
    ASSERT_TRUE(mpu6500GyroDetect(&gyro)); ASSERT_TRUE(mpu6500AccDetect(&acc));
    const uint8_t raw[]={0x10,0,0xff,0xfc,0,8,0,0,0x7f,0xff,0x80,0,0xff,0xfe};
    memcpy(registers+MPU_RA_ACCEL_XOUT_H,raw,sizeof(raw));
    ASSERT_TRUE(gyro.readFn(&gyro)); ASSERT_TRUE(acc.readFn(&acc));
    EXPECT_EQ(gyro.gyroADCRaw[0],32767); EXPECT_EQ(gyro.gyroADCRaw[1],-32768);
    EXPECT_EQ(acc.ADCRaw[0],4096); EXPECT_EQ(acc.ADCRaw[1],-4);
    failedRead=MPU_RA_ACCEL_XOUT_H;
    EXPECT_FALSE(gyro.readFn(&gyro)); EXPECT_FALSE(acc.readFn(&acc));
    EXPECT_EQ(acc.ADCRaw[0],4096);
}
