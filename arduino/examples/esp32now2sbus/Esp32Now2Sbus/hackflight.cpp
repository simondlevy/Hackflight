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
static size_t s0_idx = (0);
static size_t s1_idx = (0);
static size_t s4_idx = (0);
static size_t s3_idx = (0);
static size_t s2_idx = (0);

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
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

uint8_t esp32nowDebug_arg0(void) {
  return ((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(4))) ? (uint8_t)(0) : ((((((esp32nowByte_cpy) == ((uint8_t)(36))) ? (uint8_t)(1) : (((((s2_get)((0))) == ((uint8_t)(1))) && ((esp32nowByte_cpy) == ((uint8_t)(77)))) ? (uint8_t)(2) : (((((s2_get)((0))) == ((uint8_t)(2))) && (((esp32nowByte_cpy) == ((uint8_t)(60))) || ((esp32nowByte_cpy) == ((uint8_t)(62))))) ? (uint8_t)(3) : ((((s2_get)((0))) == ((uint8_t)(3))) ? (uint8_t)(4) : ((((s2_get)((0))) == ((uint8_t)(4))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s3_get)((0))) < ((uint8_t)(200)))) ? (uint8_t)(6) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) > ((uint8_t)(1)))) ? (uint8_t)(5) : (((((s2_get)((0))) == ((uint8_t)(5))) && (((s4_get)((0))) == ((uint8_t)(1)))) ? (uint8_t)(6) : ((uint8_t)(0)))))))))) == ((uint8_t)(5))) && (((s3_get)((0))) == ((uint8_t)(0)))) ? esp32nowByte_cpy : ((s3_get)((0))));
}

void step(void) {
  bool s0_tmp;
  bool s1_tmp;
  uint8_t s4_tmp;
  uint8_t s3_tmp;
  uint8_t s2_tmp;
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
    {(esp32nowDebug)(((esp32nowDebug_arg0)()));}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  (s4_tmp) = ((s4_gen)());
  (s3_tmp) = ((s3_gen)());
  (s2_tmp) = ((s2_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  ((s4)[s4_idx]) = (s4_tmp);
  ((s3)[s3_idx]) = (s3_tmp);
  ((s2)[s2_idx]) = (s2_tmp);
  (s0_idx) = (((s0_idx) + (1)) % (1));
  (s1_idx) = (((s1_idx) + (1)) % (1));
  (s4_idx) = (((s4_idx) + (1)) % (1));
  (s3_idx) = (((s3_idx) + (1)) % (1));
  (s2_idx) = (((s2_idx) + (1)) % (1));
}
