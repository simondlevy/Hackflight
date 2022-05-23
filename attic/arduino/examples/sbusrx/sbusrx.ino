/*
 * SBUS input tester
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>

SbusRx sbus_in(&Serial1);

void setup() {

    sbus_in.Begin(
#ifdef ESP32
          4, 14
#endif
        );

    Serial.begin(115000);
}

void loop() {

  if (sbus_in.Read()) {
    /* Display the received data */
    for (uint8_t i = 0; i < sbus_in.rx_channels().size(); i++) {
      Serial.print(sbus_in.rx_channels()[i]);
      Serial.print("\t");
    }

    /* Display lost frames and failsafe data */
    Serial.print(sbus_in.lost_frame());
    Serial.print("\t");
    Serial.println(sbus_in.failsafe());
  }
}

