/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
#include <cmath>
#include <fstream>
#include <iomanip>
extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "config/parameter_group_ids.h"
#include "sensors/sensors.h"
#include "common/utils.h"
#include "drivers/compass/compass.h"
#include "sensors/compass.h"
#include "sensors/compass_orientation.h"
#include "sensors/multi_mag.h"
#include "sensors/diagnostics.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"
#include "flight/imu.h"
}
#include "gtest/gtest.h"
namespace {
busDeviceDescriptor_t registry[6];
uint32_t clockUs, sensorMask;
bool failed[2], initFails[2], rotating, orientationAccept, allEndpoints;
unsigned inits[2], reads[2], saves;
}
extern "C" {
const busDeviceDescriptor_t *multiMagTestRegistryStart=registry;
const busDeviceDescriptor_t *multiMagTestRegistryEnd=registry+1;
uint32_t stateFlags;
uint32_t armingFlags;
mag_t mag;
uint8_t requestedSensors[SENSOR_INDEX_COUNT],detectedSensors[SENSOR_INDEX_COUNT];
fpQuaternion_t orientation;
PG_REGISTER(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 0);
timeUs_t micros(void){return clockUs;}
bool i2cIsInitialized(I2CDevice bus){return bus==I2CDEV_1||bus==I2CDEV_2;}
void sensorsSet(uint32_t mask){sensorMask|=mask;}
void sensorsClear(uint32_t mask){sensorMask&=~mask;}
void saveConfigAndNotify(void){saves++;}
void beeper(beeperMode_e){}
bool isImuReady(void){return true;}
bool gyroIsCalibrationComplete(void){return true;}
hardwareSensorStatus_e getHwGyroStatus(void){return HW_SENSOR_OK;}
void applyTailSitterAlignment(fpVector3_t *){}
bool mockInit(magDev_t *m){unsigned bus=m->busDev->busdev.i2c.i2cBus;inits[bus]++;return !initFails[bus];}
bool mockRead(magDev_t *m){unsigned bus=m->busDev->busdev.i2c.i2cBus;reads[bus]++;m->readStatus=failed[bus]?MAG_READ_ERROR:MAG_READ_READY;m->magADCRaw[0]=1024;m->magADCRaw[1]=10;m->magADCRaw[2]=10;if(rotating){const float a=reads[bus]*2.399963f;const float z=((reads[bus]*37)%101)/50.0f-1;const float r=sqrtf(1-z*z);m->magADCRaw[0]=lrintf(1024*r*cosf(a)+20);m->magADCRaw[1]=lrintf(1024*r*sinf(a)-30);m->magADCRaw[2]=lrintf(1024*z+40);}return !failed[bus];}
bool lis3mdlDetect(magDev_t *m){if(!allEndpoints&&m->busDev->busdev.i2c.address!=0x1e)return false;m->init=mockInit;m->read=mockRead;return true;}
void compassOrientationContextReset(compassOrientationContext_t *c){c->count=0;}
void compassOrientationContextPush(compassOrientationContext_t *c,const int16_t *,const fpQuaternion_t *){c->count++;}
bool compassOrientationContextDetect(const compassOrientationContext_t *,const float *,const int16_t *,float,int16_t *roll,int16_t *pitch,int16_t *yaw,float *confidence){if(!orientationAccept)return false;*roll=*pitch=*yaw=0;*confidence=10;return true;}
}
class MultiMagCoreTest:public ::testing::Test{
protected:
 void SetUp()override{
  memset(registry,0,sizeof(registry));registry[0].busType=BUSTYPE_I2C;registry[0].devHwType=DEVHW_LIS3MDL;registry[0].busdev.i2c.address=0x1e;
  memset(failed,0,sizeof(failed));memset(initFails,0,sizeof(initFails));memset(inits,0,sizeof(inits));memset(reads,0,sizeof(reads));
  multiMagTestRegistryEnd=registry+1;allEndpoints=rotating=orientationAccept=false;armingFlags=stateFlags=clockUs=saves=sensorMask=0;memset(&orientation,0,sizeof(orientation));orientation.q0=1;
  memset(multiMagConfigMutable(),0,sizeof(multiMagConfig_t));auto *c=multiMagConfigMutable();c->enabled=1;c->maxAgeMs=200;c->rejectMs=100;c->goodMs=100;c->recoveryMs=1000;c->agreementDegrees=15;c->rejectDegrees=60;c->fieldMinPercent=20;c->fieldMaxPercent=250;
  ASSERT_TRUE(multiMagInit());ASSERT_EQ(multiMagCount(),2u);
  for(unsigned slot=0;slot<MULTI_MAG_MAX_SOURCES;slot++){multiMagSourceState_t s;if(!multiMagCaptureSource(slot,clockUs,&s))continue;auto &cal=c->calibration[slot];cal.id=s.id;cal.flags=MULTI_MAG_CAL_VALID;cal.gain[0]=cal.gain[1]=cal.gain[2]=1024;}
 }
 void tick(uint32_t elapsed){clockUs+=elapsed;multiMagUpdate(clockUs);}
};
TEST_F(MultiMagCoreTest, DetectsSameModelOnBothBusesWithIndependentStableIdentity){
 multiMagSourceState_t a{},b{};ASSERT_TRUE(multiMagCaptureSource(4,clockUs,&a));ASSERT_TRUE(multiMagCaptureSource(10,clockUs,&b));EXPECT_NE(a.id,b.id);EXPECT_EQ(a.address,b.address);EXPECT_NE(a.bus,b.bus);
 tick(10000);EXPECT_TRUE(multiMagIsHealthy());EXPECT_EQ(reads[0],1u);EXPECT_EQ(reads[1],1u);
}
TEST_F(MultiMagCoreTest, FailedReadRetriesAfterFixedDelayWithoutDeferringForever){
 tick(10000);failed[0]=true;tick(10000);EXPECT_TRUE(multiMagIsHealthy());unsigned before=inits[0];failed[0]=false;
 for(unsigned i=0;i<9;i++)tick(100000);EXPECT_EQ(inits[0],before);
 tick(100001);EXPECT_EQ(inits[0],before);EXPECT_TRUE(multiMagIsHealthy());EXPECT_EQ(reads[1],reads[0]);
}
TEST_F(MultiMagCoreTest, FailedReinitializationIsRateLimitedAndRecovers){
 failed[0]=true;tick(10000);initFails[0]=true;tick(1000000);unsigned before=inits[0];
 for(unsigned i=0;i<9;i++)tick(100000);EXPECT_EQ(inits[0],before);initFails[0]=false;failed[0]=false;tick(100001);EXPECT_EQ(inits[0],before+1);
}
TEST_F(MultiMagCoreTest, FailedCalibrationKeepsPriorSuccessfulRecordsAndSavesNothing){
 multiMagConfig_t old=*multiMagConfig();compassConfigMutable()->magCalibrationTimeLimit=1;ENABLE_STATE(CALIBRATE_MAG);tick(10000);EXPECT_TRUE(multiMagIsCalibrating());
 for(unsigned i=0;i<20;i++)tick(100000);EXPECT_FALSE(multiMagIsCalibrating());EXPECT_EQ(saves,0u);EXPECT_EQ(memcmp(old.calibration,multiMagConfig()->calibration,sizeof(old.calibration)),0);
}
TEST_F(MultiMagCoreTest, PaginationUsesFixedRecordsAndBoundedCursor){
 uint8_t bytes[512];sbuf_t dst;sbufInit(&dst,bytes,bytes+sizeof(bytes));multiMagSerializeState(&dst,0,0);EXPECT_EQ(sbufPtr(&dst)-bytes,24+2*48);EXPECT_EQ(bytes[4],2);EXPECT_EQ(bytes[5],255);
 sbufInit(&dst,bytes,bytes+sizeof(bytes));multiMagSerializeState(&dst,255,0);EXPECT_EQ(sbufPtr(&dst)-bytes,24);EXPECT_EQ(bytes[4],0);
}
TEST_F(MultiMagCoreTest, LosingEveryEndpointClearsHealthAndReady){
 tick(10000);ASSERT_TRUE(multiMagIsReady());failed[0]=failed[1]=true;tick(10000);EXPECT_FALSE(multiMagIsHealthy());EXPECT_FALSE(multiMagIsReady());
}

TEST_F(MultiMagCoreTest, SuccessfulDeviceCommitsOnceWhileFailedPeerRetainsPriorCalibration){
 multiMagCalibration_t oldPeer=multiMagConfig()->calibration[10];
 rotating=orientationAccept=true;failed[1]=true;compassConfigMutable()->magCalibrationTimeLimit=3;
 ENABLE_STATE(CALIBRATE_MAG);tick(10000);
 for(unsigned i=0;i<160;i++)tick(20000);
 EXPECT_FALSE(multiMagIsCalibrating());EXPECT_EQ(saves,1u);
 EXPECT_EQ(memcmp(&oldPeer,&multiMagConfig()->calibration[10],sizeof(oldPeer)),0);
 multiMagSourceState_t primary{},peer{};ASSERT_TRUE(multiMagCaptureSource(4,clockUs,&primary));ASSERT_TRUE(multiMagCaptureSource(10,clockUs,&peer));
 EXPECT_EQ(primary.calibrationStatus,MAG_CAL_OK);EXPECT_NE(peer.calibrationStatus,MAG_CAL_OK);
 EXPECT_NEAR(multiMagConfig()->calibration[4].zero[0],20,3);EXPECT_NEAR(multiMagConfig()->calibration[4].zero[1],-30,3);
}

TEST_F(MultiMagCoreTest, ArmedReadErrorsNeverRunBlockingInitializationAndRecoverByPolling){
 tick(10000);armingFlags|=ARMED;unsigned before=inits[0];failed[0]=true;
 for(unsigned i=0;i<20;i++)tick(100000);
 EXPECT_EQ(inits[0],before);EXPECT_TRUE(multiMagIsHealthy());
 failed[0]=false;tick(10000);EXPECT_EQ(inits[0],before);
 multiMagSourceState_t source{};ASSERT_TRUE(multiMagCaptureSource(4,clockUs,&source));EXPECT_TRUE(source.flags&MULTI_MAG_READABLE);
}

TEST_F(MultiMagCoreTest, FullInventoryEmitsThreeFourRecordPagesForFrontendFixture){
 const uint8_t addresses[]={0x0c,0x0d,0x0e,0x1c,0x1e,0x2c};allEndpoints=true;
 for(unsigned i=0;i<6;i++){registry[i]=registry[0];registry[i].busdev.i2c.address=addresses[i];}
 multiMagTestRegistryEnd=registry+6;ASSERT_TRUE(multiMagInit());ASSERT_EQ(multiMagCount(),12u);tick(10000);
 std::ofstream fixture("/tmp/inav-multi-mag-pages.json");fixture<<"[";
 for(unsigned page=0;page<3;page++){
  uint8_t bytes[512];sbuf_t dst;sbufInit(&dst,bytes,bytes+sizeof(bytes));multiMagSerializeState(&dst,page*4,0);
  ASSERT_EQ(sbufPtr(&dst)-bytes,24+4*48);EXPECT_EQ(bytes[4],4);EXPECT_EQ(bytes[5],page<2?(page+1)*4:255);
  if(page)fixture<<",";fixture<<"\"";
  for(uint8_t *p=bytes;p<sbufPtr(&dst);p++)fixture<<std::hex<<std::setw(2)<<std::setfill('0')<<(unsigned)*p;
  fixture<<"\"";
 }
 fixture<<"]";
}

TEST_F(MultiMagCoreTest, InitialConfigurationFailureRemainsInInventoryAndRetriesIndependently){
 initFails[0]=true;ASSERT_TRUE(multiMagInit());ASSERT_EQ(multiMagCount(),2u);
 multiMagSourceState_t broken{},peer{};ASSERT_TRUE(multiMagCaptureSource(4,clockUs,&broken));
 EXPECT_TRUE(broken.flags&MULTI_MAG_DETECTED);EXPECT_TRUE(broken.flags&MULTI_MAG_READ_ERROR);
 unsigned before=inits[0];tick(10000);EXPECT_EQ(reads[0],0u);EXPECT_GT(reads[1],0u);EXPECT_EQ(inits[0],before);
 initFails[0]=false;tick(1000000);EXPECT_EQ(inits[0],before+1);
 ASSERT_TRUE(multiMagCaptureSource(4,clockUs,&broken));EXPECT_TRUE(broken.flags&MULTI_MAG_READABLE);EXPECT_FALSE(broken.flags&MULTI_MAG_READ_ERROR);
 ASSERT_TRUE(multiMagCaptureSource(10,clockUs,&peer));EXPECT_TRUE(peer.flags&MULTI_MAG_READABLE);
}
