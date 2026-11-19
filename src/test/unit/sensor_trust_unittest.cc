/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
#include <limits>
extern "C" {
#include "common/sensor_trust.h"
}
#include "gtest/gtest.h"

class SensorTrustTest : public ::testing::Test {
protected:
    sensorTrustMonitor_t monitor{};
    sensorTrustInput_t inputs[SENSOR_TRUST_SOURCE_COUNT]{};
    uint32_t now = 100;
    void update(uint32_t dt = 100) { now += dt; sensorTrustUpdate(&monitor, inputs, now); }
    void baseline(unsigned count = 5) {
        for (unsigned i = 0; i < count; i++) inputs[i] = {true, true, 1};
        update();
    }
    void lose(unsigned source) {
        inputs[source].confidence = 0;
        update(); update(1000);
    }
};

TEST_F(SensorTrustTest, StartupAndInactiveSourcesDoNotEmitRestoration) {
    baseline();
    EXPECT_EQ(monitor.count, 0);
    for (unsigned i = 0; i < 5; i++) EXPECT_TRUE(monitor.source[i].trusted);
}
TEST_F(SensorTrustTest, BriefDropDoesNotEmitLoss) {
    baseline(); inputs[0].confidence = 0;
    update(); inputs[0].confidence = 1;
    update(999); update(1000);
    EXPECT_EQ(monitor.count, 0);
}
TEST_F(SensorTrustTest, SustainedLossAndRecoveryAreQueuedInOrder) {
    baseline(); lose(1);
    ASSERT_EQ(monitor.count, 1);
    EXPECT_EQ(monitor.events[0].source, 1);
    EXPECT_FALSE(monitor.events[0].restored);
    inputs[1].confidence = 0.8f; update(); update(1000);
    ASSERT_EQ(monitor.count, 2);
    EXPECT_EQ(monitor.events[1].source, 1);
    EXPECT_TRUE(monitor.events[1].restored);
    update(10000); EXPECT_EQ(monitor.count, 2);
}
TEST_F(SensorTrustTest, HysteresisDoesNotRestoreAtIntermediateConfidence) {
    baseline(); lose(3);
    inputs[3].confidence = 0.6f; update(); update(10000);
    EXPECT_EQ(monitor.count, 1);
    EXPECT_FALSE(monitor.source[3].trusted);
    inputs[3].confidence = 0.8f; update();
    inputs[3].confidence = 0.7f; update(900);
    inputs[3].confidence = 0.8f; update(); update(999);
    EXPECT_EQ(monitor.count, 1);
    update(1); EXPECT_EQ(monitor.count, 2);
}
TEST_F(SensorTrustTest, AllFiveSourcesRetainIndependentEvents) {
    baseline();
    for (auto &input : inputs) input.valid = false;
    update(); update(1000);
    ASSERT_EQ(monitor.count, 5);
    for (unsigned i = 0; i < 5; i++) {
        EXPECT_EQ(monitor.events[i].source, i);
        EXPECT_FALSE(monitor.events[i].restored);
    }
}
TEST_F(SensorTrustTest, InvalidAndNonfiniteDataCauseLossEvenWithHighConfidence) {
    baseline(); inputs[0].valid = false;
    inputs[1].confidence = std::numeric_limits<float>::quiet_NaN();
    update(); update(1000);
    EXPECT_EQ(monitor.count, 2);
}
TEST_F(SensorTrustTest, SourceStartingUnhealthyEmitsLossAfterDebounce) {
    inputs[2] = {true, false, 0}; update(); update(999);
    EXPECT_EQ(monitor.count, 0);
    update(1); ASSERT_EQ(monitor.count, 1);
    EXPECT_FALSE(monitor.events[0].restored);
}
TEST_F(SensorTrustTest, DisablingSourceRemovesItsEventsButRetainsOtherSources) {
    baseline(); lose(0); lose(3);
    inputs[0].active = false; update();
    ASSERT_EQ(monitor.count, 1);
    EXPECT_EQ(monitor.events[0].source, 3);
}
TEST_F(SensorTrustTest, HiddenMessageDoesNotExpire) {
    baseline(); lose(0);
    ASSERT_NE(sensorTrustPeek(&monitor, now), nullptr);
    for (unsigned i = 0; i < 100; i++) {
        now += 1000; sensorTrustPeek(&monitor, now);
    }
    EXPECT_EQ(monitor.count, 1);
    EXPECT_EQ(monitor.visibleMs, 0u);
}
TEST_F(SensorTrustTest, DisplayCreditsOnlyVisibleFramesAndDoesNotCreditLongPause) {
    baseline(); lose(0);
    sensorTrustPeek(&monitor, now); sensorTrustSetVisible(&monitor, true);
    now += 500; sensorTrustPeek(&monitor, now);
    EXPECT_EQ(monitor.visibleMs, 500u);
    now += 1000; sensorTrustPeek(&monitor, now);
    EXPECT_EQ(monitor.visibleMs, 500u);
    sensorTrustSetVisible(&monitor, true);
    now += 10000; sensorTrustPeek(&monitor, now);
    EXPECT_EQ(monitor.visibleMs, 500u);
    for (unsigned i = 0; i < 5; i++) {
        sensorTrustSetVisible(&monitor, true);
        now += 500; sensorTrustPeek(&monitor, now);
    }
    EXPECT_EQ(monitor.count, 0);
}
TEST_F(SensorTrustTest, LongHiddenQueueIsBoundedAndKeepsLatestEvents) {
    baseline();
    for (unsigned cycle = 0; cycle < 20; cycle++) {
        inputs[0].confidence = cycle % 2 ? 1 : 0;
        update(); update(1000);
    }
    ASSERT_EQ(monitor.count, SENSOR_TRUST_EVENT_CAPACITY);
    EXPECT_FALSE(monitor.events[0].restored);
    EXPECT_TRUE(monitor.events[monitor.count - 1].restored);
}
TEST_F(SensorTrustTest, DebounceWorksAcrossMillisecondWrap) {
    baseline(); now = UINT32_MAX - 500;
    inputs[4].confidence = 0; update(0); update(1000);
    ASSERT_EQ(monitor.count, 1);
    EXPECT_EQ(monitor.events[0].source, 4);
}
