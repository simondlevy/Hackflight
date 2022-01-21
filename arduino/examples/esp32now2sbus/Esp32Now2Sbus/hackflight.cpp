#define _EXTERN

#include "hackflight.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static uint8_t esp32nowByte_cpy;
static bool s0[(1)] = {(false)};
static bool s1[(1)] = {(false)};
static uint8_t s4[(1)] = {((uint8_t)(0))};
static uint8_t s3[(1)] = {((uint8_t)(0))};
static uint8_t s2[(1)] = {((uint8_t)(0))};
static uint8_t s5[(1)] = {((uint8_t)(0))};
static uint8_t s6[(1)] = {((uint8_t)(0))};
static uint16_t s7[(1)] = {((uint16_t)(0))};
static uint16_t s8[(1)] = {((uint16_t)(0))};
static float s9[(1)] = {((float)(0.0))};
static float s10[(1)] = {((float)(0.0))};
static float s11[(1)] = {((float)(0.0))};
static float s12[(1)] = {((float)(0.0))};
static float s13[(1)] = {((float)(0.0))};
static float s14[(1)] = {((float)(0.0))};
static size_t s0_idx = (0);
static size_t s1_idx = (0);
static size_t s4_idx = (0);
static size_t s3_idx = (0);
static size_t s2_idx = (0);
static size_t s5_idx = (0);
static size_t s6_idx = (0);
static size_t s7_idx = (0);
static size_t s8_idx = (0);
static size_t s9_idx = (0);
static size_t s10_idx = (0);
static size_t s11_idx = (0);
static size_t s12_idx = (0);
static size_t s13_idx = (0);
static size_t s14_idx = (0);

bool s0_get(size_t x) {
  return (s0)[((s0_idx) + (x)) % (1)];
}

bool s1_get(size_t x) {
  return (s1)[((s1_idx) + (x)) % (1)];
}

uint8_t s4_get(size_t x) {
  return (s4)[((s4_idx) + (x)) % (1)];
}

uint8_t s3_get(size_t x) {
  return (s3)[((s3_idx) + (x)) % (1)];
}

uint8_t s2_get(size_t x) {
  return (s2)[((s2_idx) + (x)) % (1)];
}

uint8_t s5_get(size_t x) {
  return (s5)[((s5_idx) + (x)) % (1)];
}

uint8_t s6_get(size_t x) {
  return (s6)[((s6_idx) + (x)) % (1)];
}

uint16_t s7_get(size_t x) {
  return (s7)[((s7_idx) + (x)) % (1)];
}

uint16_t s8_get(size_t x) {
  return (s8)[((s8_idx) + (x)) % (1)];
}

float s9_get(size_t x) {
  return (s9)[((s9_idx) + (x)) % (1)];
}

float s10_get(size_t x) {
  return (s10)[((s10_idx) + (x)) % (1)];
}

float s11_get(size_t x) {
  return (s11)[((s11_idx) + (x)) % (1)];
}

float s12_get(size_t x) {
  return (s12)[((s12_idx) + (x)) % (1)];
}

float s13_get(size_t x) {
  return (s13)[((s13_idx) + (x)) % (1)];
}

float s14_get(size_t x) {
  return (s14)[((s14_idx) + (x)) % (1)];
}

bool s0_gen(void) {
  return !((s0_get)((0)));
}

bool s1_gen(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

uint8_t s4_gen(void) {
  return ((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (esp32nowByte_cpy) + ((uint8_t)(2)) : ((((s4_get)((0))) > ((uint8_t)(0))) ? ((s4_get)((0))) - ((uint8_t)(1)) : ((uint8_t)(0)));
}

uint8_t s3_gen(void) {
  return ((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s3_get)((0))) == ((uint8_t)(0)))) ? esp32nowByte_cpy : ((s3_get)((0))));
}

uint8_t s2_gen(void) {
  return ((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))));
}

uint8_t s5_gen(void) {
  return ((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) < ((uint8_t)(4))) ? (uint8_t)(0) : (((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) ? (s5_get)((0)) : ((((s5_get)((0))) | (esp32nowByte_cpy)) & (~(((s5_get)((0))) & (esp32nowByte_cpy)))));
}

uint8_t s6_gen(void) {
  return (((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))));
}

uint16_t s7_gen(void) {
  return (uint16_t)(esp32nowByte_cpy);
}

uint16_t s8_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0)));
}

float s9_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(1))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s9_get)((0)));
}

float s10_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(2))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s10_get)((0)));
}

float s11_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(3))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s11_get)((0)));
}

float s12_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(4))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s12_get)((0)));
}

float s13_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(5))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s13_get)((0)));
}

float s14_gen(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(6))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s14_get)((0)));
}

bool serialStart_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool esp32nowStart_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool esp32nowAddPeer_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

uint8_t esp32nowAddPeer_arg0(void) {
  return (uint8_t)(152);
}

uint8_t esp32nowAddPeer_arg1(void) {
  return (uint8_t)(205);
}

uint8_t esp32nowAddPeer_arg2(void) {
  return (uint8_t)(172);
}

uint8_t esp32nowAddPeer_arg3(void) {
  return (uint8_t)(211);
}

uint8_t esp32nowAddPeer_arg4(void) {
  return (uint8_t)(66);
}

uint8_t esp32nowAddPeer_arg5(void) {
  return (uint8_t)(60);
}

bool esp32nowRegisterReceiveCallback_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool esp32nowRead_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool delayUsec_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

uint32_t delayUsec_arg0(void) {
  return (uint32_t)(1);
}

bool esp32nowDebug_guard(void) {
  return ((!(!((s0_get)((0))))) ? true : ((s1_get)((0)))) && (((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) && ((((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) < ((uint8_t)(4))) ? (uint8_t)(0) : (((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(6))) ? (s5_get)((0)) : ((((s5_get)((0))) | (esp32nowByte_cpy)) & (~(((s5_get)((0))) & (esp32nowByte_cpy)))))) == (esp32nowByte_cpy)));
}

float esp32nowDebug_arg0(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(1))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s9_get)((0)));
}

float esp32nowDebug_arg1(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(2))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s10_get)((0)));
}

float esp32nowDebug_arg2(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(3))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s11_get)((0)));
}

float esp32nowDebug_arg3(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(4))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s12_get)((0)));
}

float esp32nowDebug_arg4(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(5))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s13_get)((0)));
}

float esp32nowDebug_arg5(void) {
  return ((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) / ((uint8_t)(2))) == ((uint8_t)(6))) ? (((float)(((((((s2_get)((0))) < ((uint8_t)(5))) ? (uint8_t)(0) : ((!(((s3_get)((0))) < ((uint8_t)(200)))) ? ((s6_get)((0))) + ((uint8_t)(1)) : ((s6_get)((0))))) % ((uint8_t)(2))) == ((uint8_t)(0))) ? ((s7_get)((0))) | (((uint16_t)(esp32nowByte_cpy)) << ((uint8_t)(8))) : ((s8_get)((0))))) - ((float)(1000.0))) / ((float)(1000.0)) : ((s14_get)((0)));
}

void step(void) {
  bool s0_tmp;
  bool s1_tmp;
  uint8_t s4_tmp;
  uint8_t s3_tmp;
  uint8_t s2_tmp;
  uint8_t s5_tmp;
  uint8_t s6_tmp;
  uint16_t s7_tmp;
  uint16_t s8_tmp;
  float s9_tmp;
  float s10_tmp;
  float s11_tmp;
  float s12_tmp;
  float s13_tmp;
  float s14_tmp;
  (esp32nowByte_cpy) = (esp32nowByte);
  if ((serialStart_guard)()) {
    {(serialStart)();}
  };
  if ((esp32nowStart_guard)()) {
    {(esp32nowStart)();}
  };
  if ((esp32nowAddPeer_guard)()) {
    {(esp32nowAddPeer)(((esp32nowAddPeer_arg0)()), ((esp32nowAddPeer_arg1)()), ((esp32nowAddPeer_arg2)()), ((esp32nowAddPeer_arg3)()), ((esp32nowAddPeer_arg4)()), ((esp32nowAddPeer_arg5)()));}
  };
  if ((esp32nowRegisterReceiveCallback_guard)()) {
    {(esp32nowRegisterReceiveCallback)();}
  };
  if ((esp32nowRead_guard)()) {
    {(esp32nowRead)();}
  };
  if ((delayUsec_guard)()) {
    {(delayUsec)(((delayUsec_arg0)()));}
  };
  if ((esp32nowDebug_guard)()) {
    {(esp32nowDebug)(((esp32nowDebug_arg0)()), ((esp32nowDebug_arg1)()), ((esp32nowDebug_arg2)()), ((esp32nowDebug_arg3)()), ((esp32nowDebug_arg4)()), ((esp32nowDebug_arg5)()));}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  (s4_tmp) = ((s4_gen)());
  (s3_tmp) = ((s3_gen)());
  (s2_tmp) = ((s2_gen)());
  (s5_tmp) = ((s5_gen)());
  (s6_tmp) = ((s6_gen)());
  (s7_tmp) = ((s7_gen)());
  (s8_tmp) = ((s8_gen)());
  (s9_tmp) = ((s9_gen)());
  (s10_tmp) = ((s10_gen)());
  (s11_tmp) = ((s11_gen)());
  (s12_tmp) = ((s12_gen)());
  (s13_tmp) = ((s13_gen)());
  (s14_tmp) = ((s14_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  ((s4)[s4_idx]) = (s4_tmp);
  ((s3)[s3_idx]) = (s3_tmp);
  ((s2)[s2_idx]) = (s2_tmp);
  ((s5)[s5_idx]) = (s5_tmp);
  ((s6)[s6_idx]) = (s6_tmp);
  ((s7)[s7_idx]) = (s7_tmp);
  ((s8)[s8_idx]) = (s8_tmp);
  ((s9)[s9_idx]) = (s9_tmp);
  ((s10)[s10_idx]) = (s10_tmp);
  ((s11)[s11_idx]) = (s11_tmp);
  ((s12)[s12_idx]) = (s12_tmp);
  ((s13)[s13_idx]) = (s13_tmp);
  ((s14)[s14_idx]) = (s14_tmp);
  (s0_idx) = (((s0_idx) + (1)) % (1));
  (s1_idx) = (((s1_idx) + (1)) % (1));
  (s4_idx) = (((s4_idx) + (1)) % (1));
  (s3_idx) = (((s3_idx) + (1)) % (1));
  (s2_idx) = (((s2_idx) + (1)) % (1));
  (s5_idx) = (((s5_idx) + (1)) % (1));
  (s6_idx) = (((s6_idx) + (1)) % (1));
  (s7_idx) = (((s7_idx) + (1)) % (1));
  (s8_idx) = (((s8_idx) + (1)) % (1));
  (s9_idx) = (((s9_idx) + (1)) % (1));
  (s10_idx) = (((s10_idx) + (1)) % (1));
  (s11_idx) = (((s11_idx) + (1)) % (1));
  (s12_idx) = (((s12_idx) + (1)) % (1));
  (s13_idx) = (((s13_idx) + (1)) % (1));
  (s14_idx) = (((s14_idx) + (1)) % (1));
}
