/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cmath>
#include <cstring>
#include <limits>
extern "C" {
#include "common/sensor_fusion.h"
}
#include "gtest/gtest.h"

class SensorFusionTest : public ::testing::Test {
protected:
    sensorFusion_t state{};
    sensorFusionSample_t samples[3]{};
    sensorFusionConfig_t config{10.0f, 2.0f, 100000};
    uint32_t now = 1000;
    void set(unsigned i, float value) {
        samples[i] = {{value, value, value}, now, true};
    }
    bool update(unsigned count = 3, unsigned axes = 3) {
        return sensorFusionUpdate(&state, samples, count, axes, now, &config);
    }
};
TEST_F(SensorFusionTest, EqualConfidenceIsArithmeticMean) {
    set(0, 10); set(1, 12); set(2, 14);
    ASSERT_TRUE(update()); EXPECT_FLOAT_EQ(state.output[0], 12);
    EXPECT_EQ(state.contributors, 3);
}
TEST_F(SensorFusionTest, IsolatedSpikeLosesConfidenceAndDoesNotPolluteAcceptedHistory) {
    for (unsigned i=0;i<3;i++) set(i,100);
    ASSERT_TRUE(update());
    now+=1000; set(0,100); set(1,100); set(2,10100);
    ASSERT_TRUE(update());
    EXPECT_LT(state.member[2].confidence, 0.00001f);
    EXPECT_LT(std::fabs(state.output[0]-100), 0.01f);
    EXPECT_FLOAT_EQ(state.member[2].accepted[0], 100);
}
TEST_F(SensorFusionTest, ConfidenceRecoversGradually) {
    for (unsigned i=0;i<3;i++) set(i,100);
    ASSERT_TRUE(update());
    now+=1000; set(0,100); set(1,100); set(2,1100); ASSERT_TRUE(update());
    now+=1000; set(0,100); set(1,100); set(2,100); ASSERT_TRUE(update());
    EXPECT_LT(state.member[2].confidence,0.01f);
    for (unsigned tick=0;tick<10000;tick++) {
        now+=1000; for (unsigned i=0;i<3;i++) set(i,100); ASSERT_TRUE(update());
    }
    EXPECT_GT(state.member[2].confidence,0.99f);
}
TEST_F(SensorFusionTest, CommonRapidMotionRetainsConfidence) {
    for (unsigned i=0;i<3;i++) set(i,100);
    ASSERT_TRUE(update());
    now+=1000; for (unsigned i=0;i<3;i++) set(i,10000);
    ASSERT_TRUE(update());
    for (unsigned i=0;i<3;i++) EXPECT_FLOAT_EQ(state.member[i].confidence,1);
    EXPECT_FLOAT_EQ(state.output[0],10000);
}
TEST_F(SensorFusionTest, TwoBarometersUseTemporalEvidenceInsteadOfArbitraryPrimaryPreference) {
    set(0,100000); set(1,100000); ASSERT_TRUE(update(2,1));
    now+=10000; set(0,101000); set(1,100000); ASSERT_TRUE(update(2,1));
    EXPECT_LT(state.member[0].confidence,0.001f);
    EXPECT_FLOAT_EQ(state.member[1].confidence,1);
    EXPECT_LT(std::fabs(state.output[0]-100000),1);
}
TEST_F(SensorFusionTest, TwoBarometersCommonHeightChangeDoesNotLoseConfidence) {
    set(0,100000); set(1,100000); ASSERT_TRUE(update(2,1));
    now+=10000; set(0,99000); set(1,99000); ASSERT_TRUE(update(2,1));
    EXPECT_FLOAT_EQ(state.member[0].confidence,1);
    EXPECT_FLOAT_EQ(state.member[1].confidence,1);
}
TEST_F(SensorFusionTest, NackNaNAndInfinityNeverEnterMean) {
    set(0,100); set(1,std::numeric_limits<float>::quiet_NaN());
    set(2,std::numeric_limits<float>::infinity());
    ASSERT_TRUE(update()); EXPECT_FLOAT_EQ(state.output[0],100);
    EXPECT_EQ(state.contributors,1);
    samples[0].valid=false; EXPECT_FALSE(update()); EXPECT_FALSE(state.healthy);
    EXPECT_FLOAT_EQ(state.output[0],100);
}
TEST_F(SensorFusionTest, OldAndFutureSamplesAreExcluded) {
    set(0,100); set(1,200); set(2,300);
    samples[1].timestampUs=now-config.maxAgeUs-1;
    samples[2].timestampUs=now+1;
    ASSERT_TRUE(update()); EXPECT_FLOAT_EQ(state.output[0],100);
    now+=config.maxAgeUs+2; EXPECT_FALSE(update());
}
TEST_F(SensorFusionTest, ClockWrapPreservesFreshnessAndConfidence) {
    now=0xfffffff0u;
    set(0,1); set(1,1); set(2,1); ASSERT_TRUE(update());
    now+=1000;
    set(0,1); set(1,1); set(2,1); ASSERT_TRUE(update());
    EXPECT_EQ(state.contributors,3); EXPECT_FLOAT_EQ(state.output[0],1);
}
TEST_F(SensorFusionTest, RepeatedConsumerCallsDoNotRestoreConfidence) {
    set(0,0); set(1,0); set(2,0); ASSERT_TRUE(update());
    now+=1000; set(0,0); set(1,0); set(2,100); ASSERT_TRUE(update());
    float confidence=state.member[2].confidence;
    for (unsigned tick=0;tick<10;tick++) { now+=100; ASSERT_TRUE(update()); }
    EXPECT_FLOAT_EQ(state.member[2].confidence,confidence);
}
TEST_F(SensorFusionTest, InvalidConfigurationFailsClosedAndLeavesOutput) {
    set(0,5); ASSERT_TRUE(update(1,1));
    config.deviation=0; EXPECT_FALSE(update(1,1)); EXPECT_FLOAT_EQ(state.output[0],5);
    config.deviation=10; EXPECT_FALSE(update(4,1)); EXPECT_FALSE(update(1,0));
    EXPECT_FALSE(update(1,4)); EXPECT_FALSE(update(0,1));
}

TEST_F(SensorFusionTest, OutOfOrderMeasurementsAreRejected) {
    set(0,10); ASSERT_TRUE(update(1,1));
    now+=1000; set(0,10); ASSERT_TRUE(update(1,1));
    samples[0].timestampUs-=1; samples[0].value[0]=999;
    EXPECT_FALSE(update(1,1)); EXPECT_FALSE(state.healthy);
    EXPECT_FLOAT_EQ(state.output[0],10);
}
TEST_F(SensorFusionTest, ExactlyMaxAgeIsUsableButOneMicrosecondOlderIsNot) {
    set(0,10); now+=config.maxAgeUs;
    ASSERT_TRUE(update(1,1));
    now++; EXPECT_FALSE(update(1,1));
}
TEST_F(SensorFusionTest, ResetClearsHistoryAndConfidence) {
    set(0,10); ASSERT_TRUE(update(1,1));
    sensorFusionReset(&state);
    EXPECT_FALSE(state.healthy); EXPECT_FALSE(state.member[0].initialized);
    EXPECT_FLOAT_EQ(state.member[0].confidence,0);
}

TEST_F(SensorFusionTest, BothBarometersWithVeryLowConfidenceDoNotProduceHealthyAverage) {
    set(0,100000); set(1,110000);
    EXPECT_FALSE(update(2,1));
    EXPECT_FALSE(state.healthy); EXPECT_EQ(state.contributors,0);
    EXPECT_LT(state.member[0].confidence,SENSOR_FUSION_MIN_CONFIDENCE);
    EXPECT_LT(state.member[1].confidence,SENSOR_FUSION_MIN_CONFIDENCE);
}

TEST_F(SensorFusionTest, RecoveryAfterLongGapDoesNotRemainLockedToAnOldValue) {
    set(0,100000); ASSERT_TRUE(update(1,1));
    samples[0].valid=false;
    now+=config.maxAgeUs+1; EXPECT_FALSE(update(1,1));
    now+=1000; set(0,99000); EXPECT_FALSE(update(1,1));
    EXPECT_FALSE(state.healthy); // Reacquisition does not restore full trust immediately.
    for (unsigned i=0;i<1000;i++) { now+=1000; set(0,99000); update(1,1); }
    EXPECT_TRUE(state.healthy); EXPECT_FLOAT_EQ(state.output[0],99000);
    EXPECT_LT(state.member[0].confidence,1);
}
TEST_F(SensorFusionTest, LongReadFailureAcrossClockWrapRetainsExpiredHistoryState) {
    now=0xffff0000u; set(0,100); ASSERT_TRUE(update(1,1));
    samples[0].valid=false;
    now+=config.maxAgeUs+1; EXPECT_FALSE(update(1,1));
    EXPECT_TRUE(state.member[0].historyExpired);
    now+=1000; set(0,500); update(1,1);
    for (unsigned i=0;i<1000;i++) { now+=1000; set(0,500); update(1,1); }
    EXPECT_TRUE(state.healthy); EXPECT_FLOAT_EQ(state.output[0],500);
}

TEST_F(SensorFusionTest, ReacquisitionDoesNotTrustPersistentDisagreementWithHealthyReference) {
    set(0,100000); set(1,100000); ASSERT_TRUE(update(2,1));
    samples[0].valid=false;
    for (unsigned i=0;i<110;i++) { now+=1000; set(1,100000); ASSERT_TRUE(update(2,1)); }
    for (unsigned i=0;i<1000;i++) {
        now+=1000; set(0,110000); set(1,100000); ASSERT_TRUE(update(2,1));
    }
    EXPECT_LT(state.member[0].confidence,SENSOR_FUSION_MIN_CONFIDENCE);
    EXPECT_FLOAT_EQ(state.output[0],100000);
}

TEST_F(SensorFusionTest, FiniteExtremeInputCannotPermanentlyPoisonPredictionHistory) {
    for (unsigned i=0;i<3;i++) set(i,std::numeric_limits<float>::max());
    update(); // An unrepresentable accumulation may be rejected, but must be recoverable.
    now+=1000; for (unsigned i=0;i<3;i++) set(i,10);
    ASSERT_TRUE(update());
    now+=1000; set(0,10); samples[1].valid=samples[2].valid=false;
    ASSERT_TRUE(update()); EXPECT_FLOAT_EQ(state.output[0],10);
}

TEST_F(SensorFusionTest, IntermediateConfidenceProducesWeightedMeanRatherThanMedianOrPlainAverage) {
    for (unsigned i=0;i<3;i++) set(i,0);
    ASSERT_TRUE(update());
    now+=1000; set(0,0); set(1,0); set(2,20);
    ASSERT_TRUE(update());
    EXPECT_FLOAT_EQ(state.member[2].confidence,0.25f);
    EXPECT_NEAR(state.output[0],20.0f/9.0f,0.0001);
    EXPECT_EQ(state.contributors,3);
}
