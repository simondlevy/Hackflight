#define _EXTERN

#include "hackflight.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static bool stream_imuGotQuaternion_cpy;
static float stream_imuQuaternionW_cpy;
static float stream_imuQuaternionX_cpy;
static float stream_imuQuaternionY_cpy;
static float stream_imuQuaternionZ_cpy;
static float stream_receiverAux1_cpy;
static float stream_receiverThrottle_cpy;
static bool stream_receiverLostSignal_cpy;
static uint8_t stream_serialByte_cpy;
static uint32_t stream_micros_cpy;
static bool stream_serialAvailable_cpy;
static float stream_receiverRoll_cpy;
static float stream_receiverPitch_cpy;
static float stream_receiverYaw_cpy;
static float stream_receiverAux2_cpy;
static bool s0[(1)] = {(false)};
static bool s1[(1)] = {(false)};
static float s4[(1)] = {((float)(0.0))};
static float s5[(1)] = {((float)(0.0))};
static bool s3[(1)] = {(false)};
static bool s2[(1)] = {(false)};
static uint8_t s8[(1)] = {((uint8_t)(0))};
static uint8_t s7[(1)] = {((uint8_t)(0))};
static uint8_t s6[(1)] = {((uint8_t)(0))};
static uint8_t s9[(1)] = {((uint8_t)(0))};
static float s10[(1)] = {((float)(0.0))};
static size_t s0_idx = (0);
static size_t s1_idx = (0);
static size_t s4_idx = (0);
static size_t s5_idx = (0);
static size_t s3_idx = (0);
static size_t s2_idx = (0);
static size_t s8_idx = (0);
static size_t s7_idx = (0);
static size_t s6_idx = (0);
static size_t s9_idx = (0);
static size_t s10_idx = (0);

bool s0_get(size_t x) {
  return (s0)[((s0_idx) + (x)) % (1)];
}

bool s1_get(size_t x) {
  return (s1)[((s1_idx) + (x)) % (1)];
}

float s4_get(size_t x) {
  return (s4)[((s4_idx) + (x)) % (1)];
}

float s5_get(size_t x) {
  return (s5)[((s5_idx) + (x)) % (1)];
}

bool s3_get(size_t x) {
  return (s3)[((s3_idx) + (x)) % (1)];
}

bool s2_get(size_t x) {
  return (s2)[((s2_idx) + (x)) % (1)];
}

uint8_t s8_get(size_t x) {
  return (s8)[((s8_idx) + (x)) % (1)];
}

uint8_t s7_get(size_t x) {
  return (s7)[((s7_idx) + (x)) % (1)];
}

uint8_t s6_get(size_t x) {
  return (s6)[((s6_idx) + (x)) % (1)];
}

uint8_t s9_get(size_t x) {
  return (s9)[((s9_idx) + (x)) % (1)];
}

float s10_get(size_t x) {
  return (s10)[((s10_idx) + (x)) % (1)];
}

bool s0_gen(void) {
  return !((s0_get)((0)));
}

bool s1_gen(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

float s4_gen(void) {
  return (stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionW_cpy) * (stream_imuQuaternionX_cpy)) + ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) - ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) + ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s4_get)((0)));
}

float s5_gen(void) {
  return (stream_imuGotQuaternion_cpy) ? ((float)(0.0)) - ((asin)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionZ_cpy)) - ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionY_cpy)))))) : ((s5_get)((0)));
}

bool s3_gen(void) {
  return ((s2_get)((0))) ? false : ((((s3_get)((0))) && (!((stream_receiverAux1_cpy) > ((float)(0.0))))) ? false : (((!((s3_get)((0)))) && ((!((s2_get)((0)))) && (((((abs)(((((stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionW_cpy) * (stream_imuQuaternionX_cpy)) + ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) - ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) + ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s4_get)((0)))) * ((float)(180.0))) / ((float)(3.1415927410125732))))) < ((float)(25.0))) && (((abs)(((((stream_imuGotQuaternion_cpy) ? ((float)(0.0)) - ((asin)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionZ_cpy)) - ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionY_cpy)))))) : ((s5_get)((0)))) * ((float)(180.0))) / ((float)(3.1415927410125732))))) < ((float)(25.0)))) && (((((((float)(0.5)) + (((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) * (((float)(0.800000011920929)) + ((((float)(0.20000000298023224)) * (((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) * ((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))))) / (((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) > ((float)(0.0))) ? (float)(0.5) : ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) < ((float)(0.0))) ? (float)(0.5) : ((float)(1.0)))) * ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) > ((float)(0.0))) ? (float)(0.5) : ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) < ((float)(0.0))) ? (float)(0.5) : ((float)(1.0))))))))) * ((float)(2.0))) - ((float)(1.0))) < ((float)(-0.9950000047683716))) && ((stream_receiverAux1_cpy) > ((float)(0.0))))))) ? true : ((s3_get)((0)))));
}

bool s2_gen(void) {
  return ((s2_get)((0))) ? true : (((s3_get)((0))) && (stream_receiverLostSignal_cpy));
}

uint8_t s8_gen(void) {
  return ((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (stream_serialByte_cpy) + ((uint8_t)(2)) : ((((s8_get)((0))) > ((uint8_t)(0))) ? ((s8_get)((0))) - ((uint8_t)(1)) : ((uint8_t)(0)));
}

uint8_t s7_gen(void) {
  return ((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))));
}

uint8_t s6_gen(void) {
  return ((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))));
}

uint8_t s9_gen(void) {
  return ((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) < ((uint8_t)(4))) ? (uint8_t)(0) : (((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) ? (s9_get)((0)) : ((((s9_get)((0))) | (stream_serialByte_cpy)) & (~(((s9_get)((0))) & (stream_serialByte_cpy)))));
}

float s10_gen(void) {
  return (stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionY_cpy)) + ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) + ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) - ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s10_get)((0)));
}

bool stream_startSerial_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool stream_startI2C_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool stream_startDsmrx_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool stream_startBhy2_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool stream_startNiclaLed_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool stream_updateUsfs_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool stream_updateDsmrx_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool stream_updateTime_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool stream_writeNiclaLed_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool stream_writeNiclaLed_arg0(void) {
  return ((stream_micros_cpy) < ((uint32_t)(2000000))) ? (((stream_micros_cpy) / ((uint32_t)(50000))) % ((uint32_t)(2))) == ((uint32_t)(0)) : (((s2_get)((0))) ? false : ((((s3_get)((0))) && (!((stream_receiverAux1_cpy) > ((float)(0.0))))) ? false : (((!((s3_get)((0)))) && ((!((s2_get)((0)))) && (((((abs)(((((stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionW_cpy) * (stream_imuQuaternionX_cpy)) + ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) - ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) + ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s4_get)((0)))) * ((float)(180.0))) / ((float)(3.1415927410125732))))) < ((float)(25.0))) && (((abs)(((((stream_imuGotQuaternion_cpy) ? ((float)(0.0)) - ((asin)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionZ_cpy)) - ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionY_cpy)))))) : ((s5_get)((0)))) * ((float)(180.0))) / ((float)(3.1415927410125732))))) < ((float)(25.0)))) && (((((((float)(0.5)) + (((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) * (((float)(0.800000011920929)) + ((((float)(0.20000000298023224)) * (((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) * ((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))))) / (((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) > ((float)(0.0))) ? (float)(0.5) : ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) < ((float)(0.0))) ? (float)(0.5) : ((float)(1.0)))) * ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) > ((float)(0.0))) ? (float)(0.5) : ((((((stream_receiverThrottle_cpy) + ((float)(1.0))) / ((float)(2.0))) - ((float)(0.5))) < ((float)(0.0))) ? (float)(0.5) : ((float)(1.0))))))))) * ((float)(2.0))) - ((float)(1.0))) < ((float)(-0.9950000047683716))) && ((stream_receiverAux1_cpy) > ((float)(0.0))))))) ? true : ((s3_get)((0))))));
}

bool stream_serialUpdate_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool stream_serialRead_guard(void) {
  return stream_serialAvailable_cpy;
}

bool stream_serialSend_guard(void) {
  return (stream_serialAvailable_cpy) && ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) && ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) < ((uint8_t)(4))) ? (uint8_t)(0) : (((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) ? (s9_get)((0)) : ((((s9_get)((0))) | (stream_serialByte_cpy)) & (~(((s9_get)((0))) & (stream_serialByte_cpy)))))) == (stream_serialByte_cpy))) && (((s7_get)((0))) < ((uint8_t)(200))));
}

uint8_t stream_serialSend_arg0(void) {
  return (uint8_t)(36);
}

uint8_t stream_serialSend_arg1(void) {
  return (uint8_t)(77);
}

uint8_t stream_serialSend_arg2(void) {
  return (uint8_t)(62);
}

uint8_t stream_serialSend_arg3(void) {
  return ((uint8_t)(4)) * (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? (uint8_t)(6) : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (uint8_t)(3) : ((uint8_t)(0))));
}

uint8_t stream_serialSend_arg4(void) {
  return ((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))));
}

uint8_t stream_serialSend_arg5(void) {
  return ((((uint8_t)(4)) * (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? (uint8_t)(6) : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (uint8_t)(3) : ((uint8_t)(0))))) | (((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0)))))) & (~((((uint8_t)(4)) * (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? (uint8_t)(6) : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (uint8_t)(3) : ((uint8_t)(0))))) & (((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0)))))));
}

uint8_t stream_serialSend_arg6(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? (uint8_t)(6) : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (uint8_t)(3) : ((uint8_t)(0)));
}

float stream_serialSend_arg7(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverThrottle_cpy : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionW_cpy) * (stream_imuQuaternionX_cpy)) + ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) - ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) + ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s4_get)((0))) : ((float)(0.0)));
}

float stream_serialSend_arg8(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverRoll_cpy : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (stream_imuGotQuaternion_cpy) ? ((float)(0.0)) - ((asin)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionZ_cpy)) - ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionY_cpy)))))) : ((s5_get)((0))) : ((float)(0.0)));
}

float stream_serialSend_arg9(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverPitch_cpy : (((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(122))) ? (stream_imuGotQuaternion_cpy) ? (atan2)((((float)(2.0)) * (((stream_imuQuaternionX_cpy) * (stream_imuQuaternionY_cpy)) + ((stream_imuQuaternionW_cpy) * (stream_imuQuaternionZ_cpy)))), (((((stream_imuQuaternionW_cpy) * (stream_imuQuaternionW_cpy)) + ((stream_imuQuaternionX_cpy) * (stream_imuQuaternionX_cpy))) - ((stream_imuQuaternionY_cpy) * (stream_imuQuaternionY_cpy))) - ((stream_imuQuaternionZ_cpy) * (stream_imuQuaternionZ_cpy)))) : ((s10_get)((0))) : ((float)(0.0)));
}

float stream_serialSend_arg10(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverYaw_cpy : ((float)(0.0));
}

float stream_serialSend_arg11(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverAux1_cpy : ((float)(0.0));
}

float stream_serialSend_arg12(void) {
  return ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((stream_serialByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s6_get)((0))) == ((uint8_t)(1))) && ((stream_serialByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s6_get)((0))) == ((uint8_t)(2))) && (((stream_serialByte_cpy) == ((uint8_t)(60))) || ((stream_serialByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s6_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s6_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s7_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s6_get)((0))) == ((uint8_t)(5))) && (((s8_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s7_get)((0))) == ((uint8_t)(0)))) ? stream_serialByte_cpy : ((s7_get)((0))))) == ((uint8_t)(121))) ? stream_receiverAux2_cpy : ((float)(0.0));
}

void step(void) {
  bool s0_tmp;
  bool s1_tmp;
  float s4_tmp;
  float s5_tmp;
  bool s3_tmp;
  bool s2_tmp;
  uint8_t s8_tmp;
  uint8_t s7_tmp;
  uint8_t s6_tmp;
  uint8_t s9_tmp;
  float s10_tmp;
  (stream_imuGotQuaternion_cpy) = (stream_imuGotQuaternion);
  (stream_imuQuaternionW_cpy) = (stream_imuQuaternionW);
  (stream_imuQuaternionX_cpy) = (stream_imuQuaternionX);
  (stream_imuQuaternionY_cpy) = (stream_imuQuaternionY);
  (stream_imuQuaternionZ_cpy) = (stream_imuQuaternionZ);
  (stream_receiverAux1_cpy) = (stream_receiverAux1);
  (stream_receiverThrottle_cpy) = (stream_receiverThrottle);
  (stream_receiverLostSignal_cpy) = (stream_receiverLostSignal);
  (stream_serialByte_cpy) = (stream_serialByte);
  (stream_micros_cpy) = (stream_micros);
  (stream_serialAvailable_cpy) = (stream_serialAvailable);
  (stream_receiverRoll_cpy) = (stream_receiverRoll);
  (stream_receiverPitch_cpy) = (stream_receiverPitch);
  (stream_receiverYaw_cpy) = (stream_receiverYaw);
  (stream_receiverAux2_cpy) = (stream_receiverAux2);
  if ((stream_startSerial_guard)()) {
    {(stream_startSerial)();}
  };
  if ((stream_startI2C_guard)()) {
    {(stream_startI2C)();}
  };
  if ((stream_startDsmrx_guard)()) {
    {(stream_startDsmrx)();}
  };
  if ((stream_startBhy2_guard)()) {
    {(stream_startBhy2)();}
  };
  if ((stream_startNiclaLed_guard)()) {
    {(stream_startNiclaLed)();}
  };
  if ((stream_updateUsfs_guard)()) {
    {(stream_updateUsfs)();}
  };
  if ((stream_updateDsmrx_guard)()) {
    {(stream_updateDsmrx)();}
  };
  if ((stream_updateTime_guard)()) {
    {(stream_updateTime)();}
  };
  if ((stream_writeNiclaLed_guard)()) {
    {(stream_writeNiclaLed)(((stream_writeNiclaLed_arg0)()));}
  };
  if ((stream_serialUpdate_guard)()) {
    {(stream_serialUpdate)();}
  };
  if ((stream_serialRead_guard)()) {
    {(stream_serialRead)();}
  };
  if ((stream_serialSend_guard)()) {
    {(stream_serialSend)(((stream_serialSend_arg0)()), ((stream_serialSend_arg1)()), ((stream_serialSend_arg2)()), ((stream_serialSend_arg3)()), ((stream_serialSend_arg4)()), ((stream_serialSend_arg5)()), ((stream_serialSend_arg6)()), ((stream_serialSend_arg7)()), ((stream_serialSend_arg8)()), ((stream_serialSend_arg9)()), ((stream_serialSend_arg10)()), ((stream_serialSend_arg11)()), ((stream_serialSend_arg12)()));}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  (s4_tmp) = ((s4_gen)());
  (s5_tmp) = ((s5_gen)());
  (s3_tmp) = ((s3_gen)());
  (s2_tmp) = ((s2_gen)());
  (s8_tmp) = ((s8_gen)());
  (s7_tmp) = ((s7_gen)());
  (s6_tmp) = ((s6_gen)());
  (s9_tmp) = ((s9_gen)());
  (s10_tmp) = ((s10_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  ((s4)[s4_idx]) = (s4_tmp);
  ((s5)[s5_idx]) = (s5_tmp);
  ((s3)[s3_idx]) = (s3_tmp);
  ((s2)[s2_idx]) = (s2_tmp);
  ((s8)[s8_idx]) = (s8_tmp);
  ((s7)[s7_idx]) = (s7_tmp);
  ((s6)[s6_idx]) = (s6_tmp);
  ((s9)[s9_idx]) = (s9_tmp);
  ((s10)[s10_idx]) = (s10_tmp);
  (s0_idx) = (((s0_idx) + (1)) % (1));
  (s1_idx) = (((s1_idx) + (1)) % (1));
  (s4_idx) = (((s4_idx) + (1)) % (1));
  (s5_idx) = (((s5_idx) + (1)) % (1));
  (s3_idx) = (((s3_idx) + (1)) % (1));
  (s2_idx) = (((s2_idx) + (1)) % (1));
  (s8_idx) = (((s8_idx) + (1)) % (1));
  (s7_idx) = (((s7_idx) + (1)) % (1));
  (s6_idx) = (((s6_idx) + (1)) % (1));
  (s9_idx) = (((s9_idx) + (1)) % (1));
  (s10_idx) = (((s10_idx) + (1)) % (1));
}
