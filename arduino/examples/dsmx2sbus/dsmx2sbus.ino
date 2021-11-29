#include <sbus.h>

SbusTx sbus_tx(&Serial1);

static uint16_t SBUS_MIN = 172;
static uint16_t SBUS_MAX = 1811;

void setup() {

  sbus_tx.Begin();
}

void loop() {

    std::array<uint16_t, 16> txvals;

    static uint16_t val;
    static int8_t dir;

    if (!val) {
        val = SBUS_MIN;
        dir = +1;
    }

    for (uint8_t k=0; k<16; ++k) {
        txvals[k] = SBUS_MIN;
    }

    txvals[4] = val;

    val += dir;

    if (val == SBUS_MAX) {
        dir = -1;
    }
    
    if (val == SBUS_MIN) {
        dir = +1;
    }
     sbus_tx.tx_channels(txvals);

    sbus_tx.Write();

    delay(10);
}

