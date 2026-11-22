#include "blackbox/blackbox.c"
static FILE *qaLog;
unsigned qaWriteSize;
void blackboxWrite(uint8_t c) { if (qaLog) fputc(c, qaLog); qaWriteSize++; }
uint8_t getMotorCount(void) { return 4; }
int getServoCount(void) { return 0; }
int getThrottleIdleValue(void) { return 1000; }
static void writeHeaders(void)
{
 fputs("H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\nH Firmware type:Cleanflight\nH Firmware revision:INAV 10.0.0 (QA fixture) MATEKH743HD\nH I interval:32\nH P interval:1/1\nH acc_1G:2048\nH gyro_scale:0x3f800000\nH features:0\nH vbatref:0\n", qaLog);
 fputs("H Field I name:loopIteration,time\nH Field I signed:0,0\nH Field I predictor:0,0\nH Field I encoding:1,1\nH Field P predictor:6,2\nH Field P encoding:9,0\n",qaLog);
 for(unsigned key=0;key<4;key++) {
  fprintf(qaLog,"H Field S %s:",blackboxFieldHeaderNames[key]);
  bool comma=false;
  for(unsigned i=0;i<ARRAYLEN(blackboxSlowFields);i++) {
   const blackboxConditionalFieldDefinition_t *field=&blackboxSlowFields[i];
   if(!testBlackboxCondition(field->condition)) continue;
   if(comma) fputc(',',qaLog); comma=true;
   if(key==0) { fputs(field->name,qaLog);if(field->fieldNameIndex>=0) fprintf(qaLog,"[%d]",field->fieldNameIndex); }
   else fprintf(qaLog,"%u",key==1?field->isSigned:key==2?field->predict:field->encode);
  }
  fputc('\n',qaLog);
 }
 /* Synthetic identity mapping; physical discovery is outside this writer fixture. */
 for(unsigned i=0;i<12;i++) fprintf(qaLog,"H mag_source_%u:%u,8,%u,%u\n",i,100+i,1+i/6,64+i%6);
 fputs("H multi_mag_units:XYZ=16ref,trust=quality&255,flags=quality>>8\n",qaLog);
}
int main(int argc,char **argv) {
 memset(blackboxConditionCache,0xff,sizeof(blackboxConditionCache));bitArrayClr(blackboxConditionCache,FLIGHT_LOG_FIELD_CONDITION_NEVER);
 if(argc>1) {
  qaLog=fopen(argv[1],"wb");if(!qaLog) return 2;
  writeHeaders();
  for(unsigned frame=0;frame<5;frame++) {
   for(unsigned i=0;i<12;i++) {
    multiMagSlowHistory.value[i][0]=-100+(int)frame;
    multiMagSlowHistory.value[i][1]=i;
    multiMagSlowHistory.value[i][2]=64;
    multiMagSlowHistory.value[i][3]=frame<2?3071:12032;
   }
   writeSlowFrame(); blackboxWrite(frame?'P':'I');
   if(!frame) {blackboxWriteUnsignedVB(0);blackboxWriteUnsignedVB(1000000);}
   else blackboxWriteSignedVB(frame==1?1000:0);
  }
  fclose(qaLog);qaLog=NULL;qaWriteSize=0;
 }
 for(unsigned i=0;i<12;i++){multiMagSlowHistory.value[i][0]=127;multiMagSlowHistory.value[i][1]=-127;multiMagSlowHistory.value[i][2]=127;multiMagSlowHistory.value[i][3]=16383;}
 writeSlowFrame();printf("zero native S plus12MAG=%u\n",qaWriteSize);qaWriteSize=0;
 slowHistory.rcModeFlags=slowHistory.rcModeFlags2=slowHistory.activeFlightModeFlags=slowHistory.stateFlags=UINT32_MAX;
 slowHistory.failsafePhase=255;slowHistory.rxSignalReceived=slowHistory.rxFlightChannelsValid=true;
 slowHistory.hwHealthStatus=INT32_MAX;slowHistory.powerSupplyImpedance=slowHistory.sagCompensatedVBat=slowHistory.rxUpdateRate=UINT16_MAX;slowHistory.activeWpNumber=255;
 for(unsigned i=0;i<3;i++)slowHistory.wind[i]=INT16_MIN;
 slowHistory.imuTemperature=slowHistory.baroTemperature=INT16_MIN;
#ifdef USE_TEMPERATURE_SENSOR
 for(unsigned i=0;i<8;i++)slowHistory.tempSensorTemperature[i]=INT16_MIN;
#endif
#ifdef USE_ESC_SENSOR
 slowHistory.escRPM=UINT32_MAX;slowHistory.escTemperature=INT8_MIN;
#endif
#ifdef USE_TERRAIN
 slowHistory.terrainAGL=slowHistory.terrainAMSL=INT32_MIN;
#endif
#ifdef USE_DRONECAN
 slowHistory.droneCANBusOffCount=UINT32_MAX;
#endif
#ifdef USE_MSP_RC_OVERRIDE
 slowHistory.mspOverrideFlags=UINT16_MAX;
#endif
 for(unsigned i=0;i<3;i++){
 multiSlowHistory.imuType[i]=8;multiSlowHistory.imuFlags[i]=127;multiSlowHistory.imuGyroTrust[i]=multiSlowHistory.imuAccTrust[i]=1000;multiSlowHistory.imuGyroAgeMs[i]=multiSlowHistory.imuAccAgeMs[i]=65535;
 }
 for(unsigned i=0;i<9;i++)multiSlowHistory.imuRawAccMg[i]=32000;
 for(unsigned i=0;i<2;i++){multiSlowHistory.baroType[i]=7;multiSlowHistory.baroFlags[i]=15;multiSlowHistory.baroTrust[i]=1000;multiSlowHistory.baroAgeMs[i]=65535;multiSlowHistory.baroAlignedPa[i]=150000;multiSlowHistory.baroTempCenti[i]=8500;multiSlowHistory.baroAltitudeCm[i]=INT32_MIN;}
 writeSlowFrame();printf("bounded full HD S plus12MAG=%u\n",qaWriteSize);qaWriteSize=0;
 for(unsigned i=0;i<12;i++) bitArrayClr(blackboxConditionCache,FLIGHT_LOG_FIELD_CONDITION_MULTI_MAG_0+i);
 writeSlowFrame();printf("bounded native full HD S=%u\n",qaWriteSize);qaWriteSize=0;
 for(unsigned i=0;i<7;i++) bitArraySet(blackboxConditionCache,FLIGHT_LOG_FIELD_CONDITION_MULTI_MAG_0+i);
 writeSlowFrame();printf("bounded full HD S plus7MAG=%u\n",qaWriteSize);return 0;
}
