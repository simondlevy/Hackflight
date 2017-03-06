#include <Arduino.h>

//#include <hackflight.hpp>
#include "naze.hpp"

//hf::Hackflight h;

void setup(void)
{
    Serial.begin(115200);
    //h.init(new hf::Naze());

    hf::Board * board = new hf::Naze();

    board->init();

    board->setLed(0, false);
    board->setLed(1, false);

    for (uint8_t i = 0; i < 10; i++) {
        board->setLed(0, true);
        board->setLed(1, false);
        board->delayMilliseconds(50);
        board->setLed(0, false);
        board->setLed(1, true);
        board->delayMilliseconds(50);
     }

    board->setLed(0, false);
    board->setLed(1, false);
}

void loop(void)
{
}
