/*
 * This file is part of INAV.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <cstring>
#include <cstdint>
extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "drivers/time.h"
#include "drivers/compass/compass.h"
bool ak8963Detect(magDev_t *);
bool ak8975Detect(magDev_t *);
bool hmc5883lDetect(magDev_t *);
bool ist8308Detect(magDev_t *);
bool ist8310Detect(magDev_t *);
bool lis2mdlDetect(magDev_t *);
bool lis3mdlDetect(magDev_t *);
bool mag3110detect(magDev_t *);
bool mlx90393Detect(magDev_t *);
bool qmc5883Detect(magDev_t *);
bool qmc5883pDetect(magDev_t *);
bool vcm5883Detect(magDev_t *);
}
#include "gtest/gtest.h"
namespace {
struct Endpoint {
    busDeviceDescriptor_t descriptor;
    busDevice_t bus;
    magDev_t mag;
    uint8_t registers[256];
    int failRead, failWrite;
    unsigned reads, deinit;
};
Endpoint endpoints[2];
unsigned registryCalls;
timeUs_t nowUs;
Endpoint &endpoint(const busDevice_t *bus) {
    for (auto &e : endpoints) if (&e.bus == bus) return e;
    ADD_FAILURE() << "Driver accessed another endpoint";
    return endpoints[0];
}
void configure(Endpoint &e, devHardwareType_e hardware) {
    memset(&e, 0, sizeof(e));
    e.failRead = e.failWrite = -1;
    e.descriptor.devicePtr = &e.bus;
    e.descriptor.busType = e.bus.busType = BUSTYPE_I2C;
    e.descriptor.devHwType = hardware;
    e.bus.descriptorPtr = &e.descriptor;
    e.mag.busDev = &e.bus;
    e.mag.usePreconfiguredBus = true;
    e.mag.magADCRaw[0] = 101;
    e.mag.magADCRaw[1] = 202;
    e.mag.magADCRaw[2] = 303;
}
struct DriverCase {
    devHardwareType_e hardware;
    bool (*detect)(magDev_t *);
    uint8_t identityRegister, identity, statusRegister, ready;
};
const DriverCase drivers[] = {
    {DEVHW_AK8963, ak8963Detect, 0x00, 0x48, 0x02, 1},
    {DEVHW_AK8975, ak8975Detect, 0x00, 0x48, 0x02, 1},
    {DEVHW_HMC5883, hmc5883lDetect, 0x0a, 'H', 0x09, 1},
    {DEVHW_IST8308, ist8308Detect, 0x00, 0x08, 0x10, 1},
    {DEVHW_IST8310_0, ist8310Detect, 0x00, 0x10, 0x02, 1},
    {DEVHW_IST8310_1, ist8310Detect, 0x00, 0x10, 0x02, 1},
    {DEVHW_LIS2MDL, lis2mdlDetect, 0x4f, 0x40, 0x67, 8},
    {DEVHW_LIS3MDL, lis3mdlDetect, 0x0f, 0x3d, 0x27, 8},
    {DEVHW_MAG3110, mag3110detect, 0x07, 0xc4, 0x00, 8},
    {DEVHW_QMC5883, qmc5883Detect, 0x0d, 0xff, 0x06, 1},
    {DEVHW_QMC5883P, qmc5883pDetect, 0x00, 0x80, 0x09, 1},
};
}
extern "C" {
busDevice_t *busDeviceInit(busType_e, devHardwareType_e, uint8_t, resourceOwner_e) {
    registryCalls++;
    return &endpoints[0].bus;
}
void busDeviceDeInit(busDevice_t *bus) { endpoint(bus).deinit++; }
void busSetSpeed(const busDevice_t *, busSpeed_e) {}
bool busReadBuf(const busDevice_t *bus, uint8_t reg, uint8_t *values, uint8_t length) {
    auto &e = endpoint(bus);
    e.reads++;
    if (e.failRead == reg) return false; // Leave destination unchanged on NACK.
    memcpy(values, e.registers + reg, length);
    return true;
}
bool busRead(const busDevice_t *bus, uint8_t reg, uint8_t *value) { return busReadBuf(bus, reg, value, 1); }
bool busWrite(const busDevice_t *bus, uint8_t reg, uint8_t value) {
    auto &e = endpoint(bus);
    if (e.failWrite == reg) return false;
    e.registers[reg] = value;
    return true;
}
bool busWriteBuf(const busDevice_t *bus, uint8_t reg, const uint8_t *data, uint8_t length) {
    auto &e = endpoint(bus);
    if (e.failWrite == reg) return false;
    memcpy(e.registers + reg, data, length);
    return true;
}
void delay(timeMs_t) {}
timeUs_t micros(void) { return nowUs; }
}
class MultiMagBusTest : public ::testing::Test {
protected:
    void SetUp() override {
        registryCalls = nowUs = 0;
        configure(endpoints[0], DEVHW_LIS3MDL);
        configure(endpoints[1], DEVHW_LIS3MDL);
    }
};
TEST_F(MultiMagBusTest, OwnContextIsNotResetOrRedirectedToGlobalRegistry) {
    auto &e = endpoints[1];
    e.bus.flags = 1234;
    EXPECT_EQ(magBusDeviceInit(&e.mag, BUSTYPE_ANY, DEVHW_LIS3MDL), &e.bus);
    EXPECT_EQ(e.bus.flags, 1234u);
    EXPECT_EQ(registryCalls, 0u);
    EXPECT_EQ(magBusDeviceInit(&e.mag, BUSTYPE_ANY, DEVHW_AK8963), nullptr);
    EXPECT_EQ(magBusDeviceInit(&e.mag, BUSTYPE_SPI, DEVHW_LIS3MDL), nullptr);
    e.descriptor.devicePtr = &endpoints[0].bus;
    EXPECT_EQ(magBusDeviceInit(&e.mag, BUSTYPE_ANY, DEVHW_LIS3MDL), nullptr);
    e.mag.usePreconfiguredBus = false;
    EXPECT_EQ(magBusDeviceInit(&e.mag, BUSTYPE_ANY, DEVHW_LIS3MDL), &endpoints[0].bus);
    EXPECT_EQ(registryCalls, 1u);
}
TEST_F(MultiMagBusTest, EachDataReadyDriverDistinguishesFreshCachedAndTransportFailure) {
    for (const auto &d : drivers) {
        SCOPED_TRACE(d.hardware);
        auto &e = endpoints[0];
        configure(e, d.hardware);
        e.registers[d.identityRegister] = d.identity;
        ASSERT_TRUE(d.detect(&e.mag));
        e.registers[d.statusRegister] = 0;
        e.mag.read(&e.mag);
        EXPECT_EQ(e.mag.readStatus, MAG_READ_NOT_READY);
        EXPECT_EQ(e.mag.magADCRaw[0], 101);
        EXPECT_EQ(e.mag.magADCRaw[1], 202);
        EXPECT_EQ(e.mag.magADCRaw[2], 303);
        e.registers[d.statusRegister] = d.ready;
        e.mag.read(&e.mag);
        EXPECT_EQ(e.mag.readStatus, MAG_READ_READY);
        e.failRead = d.statusRegister;
        EXPECT_FALSE(e.mag.read(&e.mag));
        EXPECT_EQ(e.mag.readStatus, MAG_READ_ERROR);
        EXPECT_EQ(registryCalls, 0u);
    }
}
TEST_F(MultiMagBusTest, SameModelEndpointsAndFactoryTrimsAreIndependent) {
    for (unsigned i=0; i<2; i++) {
        auto &e = endpoints[i];
        configure(e, DEVHW_AK8963);
        e.registers[0] = 0x48;
        memset(e.registers + 0x10, 128 + 64*i, 3);
        ASSERT_TRUE(ak8963Detect(&e.mag));
        ASSERT_TRUE(e.mag.init(&e.mag));
        e.registers[2] = 1;
        e.registers[3] = 100;
    }
    EXPECT_TRUE(endpoints[0].mag.read(&endpoints[0].mag));
    EXPECT_TRUE(endpoints[1].mag.read(&endpoints[1].mag));
    EXPECT_EQ(endpoints[0].mag.magADCRaw[0], -100);
    EXPECT_EQ(endpoints[1].mag.magADCRaw[0], -125);
    endpoints[1].failRead = 2;
    EXPECT_FALSE(endpoints[1].mag.read(&endpoints[1].mag));
    EXPECT_TRUE(endpoints[0].mag.read(&endpoints[0].mag));
    EXPECT_EQ(endpoints[0].mag.readStatus, MAG_READ_READY);
}
TEST_F(MultiMagBusTest, Ak8975DataNackCannotBeOverwrittenBySuccessfulStatusRead) {
    auto &e = endpoints[0];
    configure(e, DEVHW_AK8975);
    e.registers[0] = 0x48;
    ASSERT_TRUE(ak8975Detect(&e.mag));
    e.registers[2] = 1;
    e.failRead = 3;
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_ERROR);
    EXPECT_EQ(e.mag.magADCRaw[0], 101);
}
TEST_F(MultiMagBusTest, ConfigurationNackIsStrictOnlyForMultiMode) {
    auto &e = endpoints[0];
    e.registers[0x0f] = 0x3d;
    ASSERT_TRUE(lis3mdlDetect(&e.mag));
    e.failWrite = 0x21;
    EXPECT_FALSE(e.mag.init(&e.mag));
    e.mag.usePreconfiguredBus = false;
    EXPECT_TRUE(e.mag.init(&e.mag));
}
TEST_F(MultiMagBusTest, HmcLockedAndOverflowSamplesAreNotFresh) {
    auto &e = endpoints[0];
    configure(e, DEVHW_HMC5883);
    e.registers[0x0a] = 'H';
    ASSERT_TRUE(hmc5883lDetect(&e.mag));
    e.registers[9] = 3;
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_NOT_READY);
    e.registers[9] = 1;
    e.registers[3] = 0xf0; // -4096
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_ERROR);
    EXPECT_EQ(e.mag.magADCRaw[0], 101);
}
TEST_F(MultiMagBusTest, MlxReadMeasurementErrorIsNotNewData) {
    auto &e = endpoints[0];
    configure(e, DEVHW_MLX90393);
    e.registers[0xf0] = 4; // Reset identification reply.
    ASSERT_TRUE(mlx90393Detect(&e.mag));
    e.registers[0x4e] = 0x92; // Burst, ERROR (DRDY low), six data bytes.
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_NOT_READY);
    EXPECT_EQ(e.mag.magADCRaw[0], 101);
    e.registers[0x4e] = 0x82;
    EXPECT_TRUE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_READY);
    e.registers[0x4e] = 0x80; // Wrong response length.
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_ERROR);
}
TEST_F(MultiMagBusTest, VcmPacesReadoutsAndDetectsStandbyWithClockWrap) {
    auto &e = endpoints[0];
    configure(e, DEVHW_VCM5883);
    e.registers[0x0c] = 0x82;
    ASSERT_TRUE(vcm5883Detect(&e.mag));
    nowUs = UINT32_MAX - 5000;
    ASSERT_TRUE(e.mag.init(&e.mag));
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_NOT_READY);
    nowUs += 10000;
    EXPECT_TRUE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_READY);
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_NOT_READY);
    nowUs += 10000;
    e.registers[0x0a] = 0x40;
    EXPECT_FALSE(e.mag.read(&e.mag));
    EXPECT_EQ(e.mag.readStatus, MAG_READ_ERROR);
}
