/*
 * ESP32 NOW support
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <string.h>
#include <esp_now.h>
#include <WiFi.h>
#include "arduino_debugger.hpp"

#define _EXTERN
#include "hackflight.h"

static uint8_t _msg[256];
static uint8_t _msglen;

static void _data_receive_callback(const uint8_t * macaddr, const uint8_t *data, int len)
{
    memcpy(_msg, data, len);

    _msglen = len;

    delayMicroseconds(1);
}

void esp32nowRegisterReceiveCallback(void)
{
  esp_now_register_recv_cb(_data_receive_callback);
}


void esp32nowRead(void)
{
    static uint8_t _msgidx;
    esp32nowByte = _msg[_msgidx++];
    _msgidx = _msglen > 0 ? _msgidx % _msglen : 0;
}
