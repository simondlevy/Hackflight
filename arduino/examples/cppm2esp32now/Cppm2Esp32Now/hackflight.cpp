#define _EXTERN

#include "hackflight.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static bool receiverGotNewFrame_cpy;
static uint16_t receiverChan1_cpy;
static uint16_t receiverChan2_cpy;
static uint16_t receiverChan3_cpy;
static uint16_t receiverChan4_cpy;
static uint16_t receiverChan5_cpy;
static uint16_t receiverChan6_cpy;
static bool s0[(1)] = {(false)};
static bool s1[(1)] = {(false)};
static size_t s0_idx = (0);
static size_t s1_idx = (0);

bool s0_get(size_t x) {
  return (s0)[((s0_idx) + (x)) % (1)];
}

bool s1_get(size_t x) {
  return (s1)[((s1_idx) + (x)) % (1)];
}

bool s0_gen(void) {
  return !((s0_get)((0)));
}

bool s1_gen(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool serialStart_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool cppmStart_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

uint8_t cppmStart_arg0(void) {
  return (uint8_t)(4);
}

uint8_t cppmStart_arg1(void) {
  return (uint8_t)(6);
}

bool esp32nowStart_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

bool esp32nowAddPeer_guard(void) {
  return !((!(!((s0_get)((0))))) ? true : ((s1_get)((0))));
}

uint8_t esp32nowAddPeer_arg0(void) {
  return (uint8_t)(216);
}

uint8_t esp32nowAddPeer_arg1(void) {
  return (uint8_t)(160);
}

uint8_t esp32nowAddPeer_arg2(void) {
  return (uint8_t)(29);
}

uint8_t esp32nowAddPeer_arg3(void) {
  return (uint8_t)(84);
}

uint8_t esp32nowAddPeer_arg4(void) {
  return (uint8_t)(135);
}

uint8_t esp32nowAddPeer_arg5(void) {
  return (uint8_t)(68);
}

bool cppmUpdate_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

bool cppmGet_guard(void) {
  return receiverGotNewFrame_cpy;
}

bool esp32nowPrepareToSend_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

uint8_t esp32nowPrepareToSend_arg0(void) {
  return (uint8_t)(216);
}

uint8_t esp32nowPrepareToSend_arg1(void) {
  return (uint8_t)(160);
}

uint8_t esp32nowPrepareToSend_arg2(void) {
  return (uint8_t)(29);
}

uint8_t esp32nowPrepareToSend_arg3(void) {
  return (uint8_t)(84);
}

uint8_t esp32nowPrepareToSend_arg4(void) {
  return (uint8_t)(135);
}

uint8_t esp32nowPrepareToSend_arg5(void) {
  return (uint8_t)(68);
}

bool commsSend_guard(void) {
  return (!(!((s0_get)((0))))) ? true : ((s1_get)((0)));
}

uint8_t commsSend_arg0(void) {
  return (uint8_t)(60);
}

uint8_t commsSend_arg1(void) {
  return (uint8_t)(12);
}

uint8_t commsSend_arg2(void) {
  return (uint8_t)(200);
}

float commsSend_arg3(void) {
  return (float)(receiverChan1_cpy);
}

float commsSend_arg4(void) {
  return (float)(receiverChan2_cpy);
}

float commsSend_arg5(void) {
  return (float)(receiverChan3_cpy);
}

float commsSend_arg6(void) {
  return (float)(receiverChan4_cpy);
}

float commsSend_arg7(void) {
  return (float)(receiverChan5_cpy);
}

float commsSend_arg8(void) {
  return (float)(receiverChan6_cpy);
}

void step(void) {
  bool s0_tmp;
  bool s1_tmp;
  (receiverGotNewFrame_cpy) = (receiverGotNewFrame);
  (receiverChan1_cpy) = (receiverChan1);
  (receiverChan2_cpy) = (receiverChan2);
  (receiverChan3_cpy) = (receiverChan3);
  (receiverChan4_cpy) = (receiverChan4);
  (receiverChan5_cpy) = (receiverChan5);
  (receiverChan6_cpy) = (receiverChan6);
  if ((serialStart_guard)()) {
    {(serialStart)();}
  };
  if ((cppmStart_guard)()) {
    {(cppmStart)(((cppmStart_arg0)()), ((cppmStart_arg1)()));}
  };
  if ((esp32nowStart_guard)()) {
    {(esp32nowStart)();}
  };
  if ((esp32nowAddPeer_guard)()) {
    {(esp32nowAddPeer)(((esp32nowAddPeer_arg0)()), ((esp32nowAddPeer_arg1)()), ((esp32nowAddPeer_arg2)()), ((esp32nowAddPeer_arg3)()), ((esp32nowAddPeer_arg4)()), ((esp32nowAddPeer_arg5)()));}
  };
  if ((cppmUpdate_guard)()) {
    {(cppmUpdate)();}
  };
  if ((cppmGet_guard)()) {
    {(cppmGet)();}
  };
  if ((esp32nowPrepareToSend_guard)()) {
    {(esp32nowPrepareToSend)(((esp32nowPrepareToSend_arg0)()), ((esp32nowPrepareToSend_arg1)()), ((esp32nowPrepareToSend_arg2)()), ((esp32nowPrepareToSend_arg3)()), ((esp32nowPrepareToSend_arg4)()), ((esp32nowPrepareToSend_arg5)()));}
  };
  if ((commsSend_guard)()) {
    {(commsSend)(((commsSend_arg0)()), ((commsSend_arg1)()), ((commsSend_arg2)()), ((commsSend_arg3)()), ((commsSend_arg4)()), ((commsSend_arg5)()), ((commsSend_arg6)()), ((commsSend_arg7)()), ((commsSend_arg8)()));}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  (s0_idx) = (((s0_idx) + (1)) % (1));
  (s1_idx) = (((s1_idx) + (1)) % (1));
}
