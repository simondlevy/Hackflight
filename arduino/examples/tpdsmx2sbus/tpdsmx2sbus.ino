/*
 * Spektrum DSMX to SBUS transducer for TinyPICO
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>
#include <DSMRX.h>

static const uint8_t  SBUS_CHANNELS = 16;
static const uint16_t SBUS_MIN = 172;
static const uint16_t SBUS_MAX = 1811;

static const uint8_t SBUS_RX_PIN = 21; // unused
static const uint8_t SBUS_TX_PIN = 22; 

static const uint8_t DSMX_RX_PIN = 4;
static const uint8_t DSMX_TX_PIN = 14; // unused
static const uint8_t DSMX_CHANNELS = 8;

SbusTx sbus_out(&Serial1, SBUS_RX_PIN, SBUS_TX_PIN);

DSM2048 dsmx_in;

static void coreTask(void * params)
{
    while (true) {
      
        if (Serial2.available()) {
           dsmx_in.handleSerialEvent(Serial2.read(), micros()); 
        }

        delay(1);
    } 
}

static uint16_t scale(float txval)
{
    return ((txval + 1) / 2) * (SBUS_MAX - SBUS_MIN) + SBUS_MIN;
}

void setup() {

    sbus_out.Begin();

    Serial.begin(115000);

    Serial2.begin(115000, SERIAL_8N1, DSMX_RX_PIN, DSMX_TX_PIN);
    TaskHandle_t task;
    xTaskCreatePinnedToCore(coreTask, "Task", 10000, NULL, 1, &task, 0); 
}

void loop() {

    static float dsmxvals[DSMX_CHANNELS];

    if (dsmx_in.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (dsmx_in.gotNewFrame()) {

        dsmx_in.getChannelValues(dsmxvals, DSMX_CHANNELS);
    }

    std::array<uint16_t, SBUS_CHANNELS> sbusvals;

    sbusvals[0] = scale(dsmxvals[0]);
    sbusvals[1] = scale(dsmxvals[1]);
    sbusvals[2] = scale(dsmxvals[2]);
    sbusvals[3] = scale(dsmxvals[3]);
    sbusvals[4] = scale(dsmxvals[6]);

    Serial.println(dsmxvals[6]);

    for (uint8_t k=5; k<SBUS_CHANNELS; ++k) {
        sbusvals[k] = SBUS_MIN;
    }

    sbus_out.tx_channels(sbusvals);
    sbus_out.Write();

    delay(5);
}

