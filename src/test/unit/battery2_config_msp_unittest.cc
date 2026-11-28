/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "sensors/battery2_config_msp.h"
#include "fc/msp_setting_value.h"
}
#include "gtest/gtest.h"
class Battery2ConfigMspTest : public ::testing::Test {
protected:
 uint8_t bytes[32];
 battery2Config_t previous{},wanted{};
 void SetUp() override {
  previous.voltageScale=1000;previous.currentScale=2;previous.currentI2cAddress=69;
  previous.voltageEnabled=previous.currentEnabled=1;
  previous.voltageSensor=previous.currentSensor=BATTERY2_SENSOR_I2C;
  wanted=previous;wanted.voltageScale=1234;wanted.voltageOffset=-55;wanted.currentScale=321;wanted.currentOffset=-7;
  wanted.cells=6;wanted.cellDetect=430;wanted.cellWarning=350;wanted.cellCritical=330;
  wanted.voltageEnabled=0;wanted.voltageSensor=BATTERY2_SENSOR_CAN;wanted.currentSensor=BATTERY2_SENSOR_ADC;
  wanted.canBatteryId=23;wanted.currentI2cAddress=68;
 }
 void encode(){
  sbuf_t dst;sbufInit(&dst,bytes,bytes+sizeof(bytes));
  sbufWriteU16(&dst,wanted.voltageScale);sbufWriteU16(&dst,wanted.voltageOffset);
  sbufWriteU16(&dst,wanted.currentScale);sbufWriteU16(&dst,wanted.currentOffset);sbufWriteU8(&dst,wanted.cells);
  sbufWriteU16(&dst,wanted.cellDetect);sbufWriteU16(&dst,wanted.cellWarning);sbufWriteU16(&dst,wanted.cellCritical);
  sbufWriteU8(&dst,wanted.voltageEnabled);sbufWriteU8(&dst,wanted.voltageSensor);sbufWriteU8(&dst,wanted.currentEnabled);
  sbufWriteU8(&dst,wanted.currentSensor);sbufWriteU8(&dst,wanted.canBatteryId);sbufWriteU8(&dst,wanted.currentI2cAddress);
 }
 bool decode(unsigned length,battery2Config_t *out){sbuf_t src;sbufInit(&src,bytes,bytes+length);return battery2ConfigDecode(&src,&previous,out);}
};
TEST_F(Battery2ConfigMspTest, AllSupportedPacketVersionsPreserveTheirCompleteFieldContract){
 encode();
 for(unsigned length:{15,20,21}){
  battery2Config_t out{};ASSERT_TRUE(decode(length,&out));
  EXPECT_EQ(out.voltageScale,wanted.voltageScale);EXPECT_EQ(out.voltageOffset,wanted.voltageOffset);
  EXPECT_EQ(out.currentScale,wanted.currentScale);EXPECT_EQ(out.currentOffset,wanted.currentOffset);
  EXPECT_EQ(out.cells,wanted.cells);EXPECT_EQ(out.cellDetect,wanted.cellDetect);EXPECT_EQ(out.cellWarning,wanted.cellWarning);EXPECT_EQ(out.cellCritical,wanted.cellCritical);
  EXPECT_EQ(out.voltageEnabled,length>=20?wanted.voltageEnabled:previous.voltageEnabled);
  EXPECT_EQ(out.voltageSensor,length>=20?wanted.voltageSensor:previous.voltageSensor);
  EXPECT_EQ(out.currentEnabled,length>=20?wanted.currentEnabled:previous.currentEnabled);
  EXPECT_EQ(out.currentSensor,length>=20?wanted.currentSensor:previous.currentSensor);
  EXPECT_EQ(out.canBatteryId,length>=20?wanted.canBatteryId:previous.canBatteryId);
  EXPECT_EQ(out.currentI2cAddress,length==21?wanted.currentI2cAddress:previous.currentI2cAddress);
 }
}
TEST_F(Battery2ConfigMspTest, TruncatedAndUnsupportedLengthsNeverMutateDestination){
 encode();
 for(unsigned length=0;length<32;length++)if(length!=15&&length!=20&&length!=21){
  battery2Config_t out;memset(&out,0x5a,sizeof(out));auto old=out;
  EXPECT_FALSE(decode(length,&out));EXPECT_EQ(memcmp(&out,&old,sizeof(out)),0);
 }
}
TEST_F(Battery2ConfigMspTest, InvalidConfigNeverPartiallyMutatesDestination){
 const unsigned invalidBytes[]={0,4,8,9,11,13,15,16,17,18,20};
 for(unsigned offset:invalidBytes){
  encode();
  if(offset==0||offset==4){bytes[offset]=bytes[offset+1]=0;}
  else if(offset==9||offset==11||offset==13){bytes[offset]=bytes[offset+1]=0;}
  else bytes[offset]=255;
  auto out=previous;EXPECT_FALSE(decode(21,&out))<<offset;EXPECT_EQ(memcmp(&out,&previous,sizeof(out)),0);
 }
}
TEST_F(Battery2ConfigMspTest, PointSettingWritesEnforceNonzeroScaleBoundsWithoutMutatingOnFailure){
 for(unsigned value:{0,1,65535}){
  bytes[0]=value;bytes[1]=value>>8;uint16_t out=321;sbuf_t src;sbufInit(&src,bytes,bytes+2);
  EXPECT_EQ(mspReadSettingU16(&src,1,65535,&out),value!=0);
  EXPECT_EQ(out,value?value:321);
 }
 uint16_t out=321;sbuf_t src;sbufInit(&src,bytes,bytes+1);
 EXPECT_FALSE(mspReadSettingU16(&src,1,65535,&out));EXPECT_EQ(out,321);
}
