#include <sbus.h>

SbusTx sbus_tx(&Serial1);

void setup() {

  sbus_tx.Begin();
}

void loop() {

    std::array<uint16_t, 16> txvals;

    static uint16_t val;
    static int8_t dir;

    if (!val) {
        val = 172;
        dir = +1;
    }

    for (uint8_t k=0; k<16; ++k) {
        txvals[k] = 172;
    }

    txvals[4] = val;

    val += dir;

    if (val == 1811) {
        dir = -1;
    }
    
    if (val == 172) {
        dir = +1;
    }
     sbus_tx.tx_channels(txvals);

    sbus_tx.Write();

    delay(10);
}

