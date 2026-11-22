/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cmath>
#include <cstring>
extern "C" {
#include "common/mag_fusion.h"
}
#include "gtest/gtest.h"
class MagFusionTest : public ::testing::Test {
protected:
    magFusion_t fusion{};
    magFusionSample_t samples[MULTI_MAG_MAX_SOURCES]{};
    magFusionConfig_t config{200000,100000,100000,15,60,1,200,2500};
    uint32_t now=1000000;
    void sample(unsigned slot,uint32_t id,float x,float y=0,float z=0) {
        samples[slot]={};samples[slot].id=id;samples[slot].timestampUs=now;
        samples[slot].valid=true;samples[slot].field[0]=x;samples[slot].field[1]=y;samples[slot].field[2]=z;
    }
    bool update(){return magFusionUpdate(&fusion,samples,now,&config);}
};
TEST_F(MagFusionTest, AllPositiveWeightsParticipateAndCachedSamplesDoNotRecoverTrust) {
    sample(0,5,1024);sample(1,6,1024);ASSERT_TRUE(update());
    fusion.member[1].confidence=0.001f;
    ASSERT_TRUE(update());EXPECT_EQ(fusion.contributors,2);EXPECT_FLOAT_EQ(fusion.member[1].confidence,0.001f);
}
TEST_F(MagFusionTest, AllZeroFallbackUsesLastReferenceAndContinuesItsNewMeasurements) {
    sample(0,9,1024);sample(1,1,1024);ASSERT_TRUE(update());
    EXPECT_EQ(fusion.referenceId,1u);
    for(auto &m:fusion.member){m.confidence=0;m.quarantined=true;}
    now+=10000;sample(0,9,900);sample(1,1,950);
    ASSERT_TRUE(update());EXPECT_EQ(fusion.fallbackId,1u);EXPECT_EQ(fusion.contributors,1);EXPECT_FLOAT_EQ(fusion.output[0],950);
    now+=10000;sample(1,1,975);ASSERT_TRUE(update());EXPECT_FLOAT_EQ(fusion.output[0],975);
    samples[1].valid=false;ASSERT_TRUE(update());EXPECT_EQ(fusion.fallbackId,9u);
    samples[0].valid=false;EXPECT_FALSE(update());EXPECT_FALSE(fusion.healthy);EXPECT_EQ(fusion.contributors,0);
}
TEST_F(MagFusionTest, StaleNonfiniteZeroAndBackwardSamplesAreNeverFallbackEligible) {
    sample(0,1,1024);ASSERT_TRUE(update());now+=config.maxAgeUs+1;
    EXPECT_FALSE(update());sample(0,1,NAN);EXPECT_FALSE(update());sample(0,1,0);EXPECT_FALSE(update());
    sample(0,1,1024);samples[0].timestampUs=999999;EXPECT_FALSE(update());
}
TEST_F(MagFusionTest, ConfidenceFallsForOneOutlierAndRecoversAfterSustainedAgreement) {
    for(unsigned i=0;i<3;i++)sample(i,i+1,1024);ASSERT_TRUE(update());
    now+=10000;sample(2,3,0,1024);ASSERT_TRUE(update());EXPECT_GT(fusion.member[2].confidence,0);EXPECT_LT(fusion.member[2].confidence,0.1f);
    now+=110000;for(unsigned i=0;i<2;i++)sample(i,i+1,1024);sample(2,3,0,1024);ASSERT_TRUE(update());EXPECT_FLOAT_EQ(fusion.member[2].confidence,0);EXPECT_EQ(fusion.contributors,2);
    now+=10000;for(unsigned i=0;i<3;i++)sample(i,i+1,1024);ASSERT_TRUE(update());EXPECT_FLOAT_EQ(fusion.member[2].confidence,0);
    now+=110000;for(unsigned i=0;i<3;i++)sample(i,i+1,1024);ASSERT_TRUE(update());EXPECT_GT(fusion.member[2].confidence,0);EXPECT_EQ(fusion.contributors,3);
}
TEST_F(MagFusionTest, TwoSensorsDisagreeWithoutEvidenceNeitherIsArbitrarilyTrusted) {
    sample(0,1,1024);sample(1,2,0,1024);ASSERT_TRUE(update());
    EXPECT_FLOAT_EQ(fusion.member[0].confidence,0);EXPECT_FLOAT_EQ(fusion.member[1].confidence,0);EXPECT_NE(fusion.fallbackId,0u);
}
TEST_F(MagFusionTest, TimestampWrapWorksAndInvalidPolicyCannotProduceOutput) {
    now=UINT32_MAX-10000;sample(0,1,1024);ASSERT_TRUE(update());now+=20000;sample(0,1,1024);ASSERT_TRUE(update());
    config.recoverySeconds=NAN;EXPECT_FALSE(update());EXPECT_FALSE(fusion.healthy);
}
TEST_F(MagFusionTest, ReadOutageRelearnsEarthReferenceAndAllowsRecovery) {
    sample(0,1,1024);samples[0].earthValid=true;samples[0].earthField[0]=1024;ASSERT_TRUE(update());
    samples[0].valid=false;EXPECT_FALSE(update());EXPECT_FALSE(fusion.member[0].earthReferenceValid);
    for(unsigned n=0;n<4;n++) {
        now+=100000;sample(0,1,0,1024);samples[0].earthValid=true;samples[0].earthField[1]=1024;ASSERT_TRUE(update());
    }
    EXPECT_GT(fusion.member[0].confidence,0);EXPECT_EQ(fusion.fallbackId,0u);
}
