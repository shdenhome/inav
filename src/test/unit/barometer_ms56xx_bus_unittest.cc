/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer_ms56xx.h"
int8_t ms56xx_crc(uint16_t *prom);
}
#include "gtest/gtest.h"
namespace {
busDevice_t bus;
uint16_t prom[8];
int failedRegister;
bool resetFailure, temperatureConversion;
unsigned released;
}
extern "C" {
busDevice_t *busDeviceInit(busType_e, devHardwareType_e, uint8_t, resourceOwner_e) { return &bus; }
void busDeviceDeInit(busDevice_t *) { released++; }
void busSetSpeed(const busDevice_t *, busSpeed_e) {}
void delay(timeMs_t) {}
bool busRead(const busDevice_t *, uint8_t reg, uint8_t *value) {
    *value=prom[0]>>8; return reg!=failedRegister;
}
bool busReadBuf(const busDevice_t *, uint8_t reg, uint8_t *data, uint8_t length) {
    if (reg==failedRegister) return false;
    if (reg>=0xa0 && length==2) {
        unsigned i=(reg-0xa0)/2;
        data[0]=prom[i]>>8; data[1]=prom[i]&255;
    } else if (reg==0 && length==3) {
        uint32_t value=temperatureConversion ? 8569150 : 9085466;
        data[0]=value>>16; data[1]=value>>8; data[2]=value;
    } else return false;
    return true;
}
bool busWrite(const busDevice_t *, uint8_t reg, uint8_t) {
    if (reg==0x1e) return !resetFailure;
    temperatureConversion=(reg&0x10)!=0;
    return true;
}
}
class Ms56xxBusTest : public ::testing::Test {
protected:
    baroDev_t dev{};
    void SetUp() override {
        const uint16_t values[]={0,40127,36924,23317,23282,33464,28312,0};
        memcpy(prom,values,sizeof(prom));
        for (unsigned crc=0;crc<16;crc++) {
            prom[7]=crc;
            if (ms56xx_crc(prom)==0) break;
        }
        failedRegister=-1; resetFailure=false; released=0;
    }
};
TEST_F(Ms56xxBusTest, DatasheetPressureAndTemperatureRemainUnchanged) {
    ASSERT_TRUE(ms5611Detect(&dev));
    ASSERT_TRUE(dev.start_ut(&dev)); ASSERT_TRUE(dev.get_ut(&dev));
    ASSERT_TRUE(dev.start_up(&dev)); ASSERT_TRUE(dev.get_up(&dev));
    int32_t pressure=0,temperature=0;
    ASSERT_TRUE(dev.calculate(&dev,&pressure,&temperature));
    EXPECT_EQ(pressure,100009); EXPECT_EQ(temperature,2007);
}
TEST_F(Ms56xxBusTest, DataNackPropagatesAndDoesNotOverwriteLastMeasurement) {
    ASSERT_TRUE(ms5611Detect(&dev));
    ASSERT_TRUE(dev.start_ut(&dev)); ASSERT_TRUE(dev.get_ut(&dev));
    ASSERT_TRUE(dev.start_up(&dev)); ASSERT_TRUE(dev.get_up(&dev));
    failedRegister=0;
    EXPECT_FALSE(dev.get_ut(&dev)); EXPECT_FALSE(dev.get_up(&dev));
    int32_t pressure=0,temperature=0;
    ASSERT_TRUE(dev.calculate(&dev,&pressure,&temperature));
    EXPECT_EQ(pressure,100009); EXPECT_EQ(temperature,2007);
}
TEST_F(Ms56xxBusTest, CalibrationReadFailureAbortsDetectionAndReleasesBus) {
    failedRegister=0xa4;
    EXPECT_FALSE(ms5611Detect(&dev)); EXPECT_EQ(released,1u);
}
TEST_F(Ms56xxBusTest, ResetWriteFailureAbortsDetectionAndReleasesBus) {
    resetFailure=true;
    EXPECT_FALSE(ms5611Detect(&dev)); EXPECT_EQ(released,1u);
}
