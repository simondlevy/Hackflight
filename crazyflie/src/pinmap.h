#pragma once

#if defined(TEENSYDUINO)

static const uint8_t PIN_FLOWDECK_CS = 10;

#else

enum {

    PIN_LED_BLUE_L,
    PIN_LED_GREEN_L,
    PIN_LED_RED_L,
    PIN_LED_GREEN_R,
    PIN_LED_RED_R,

    PIN_DECK_RX1,
    PIN_DECK_TX1,
    PIN_DECK_SDA,
    PIN_DECK_SCL,
    PIN_DECK_IO1,
    PIN_DECK_IO2,
    PIN_DECK_IO3,
    PIN_DECK_IO4,
    PIN_DECK_TX2,
    PIN_DECK_RX2,
    PIN_DECK_SCK,
    PIN_DECK_MISO,
    PIN_DECK_MOSI,

    PIN_BOLT_GYRO_CS,
    PIN_BOLT_ACCEL_CS,
};

static const uint8_t PIN_FLOWDECK_CS = PIN_DECK_IO3;

#endif


