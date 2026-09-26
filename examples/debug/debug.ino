#include <hackflight.h>
#include <firmware/debugger.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

static bool armed;
static bool is_down;
static bool was_down;

void serialEvent3()
{
    while (Serial3.available()) {

        static hf::MspParser parser_;

        parser_ = hf::MspParser::Parse(parser_, Serial3.read());

        if (hf::MspParser::GetId(parser_) == kMspSetChannels) {

            is_down = hf::MspParser::GetShort(parser_, 4) > 0;
        }
    }
}

void setup()
{
    Serial3.begin(115200);

    armed = false;
    was_down = true;

    delay(3000);
}

void loop()
{
    if (!is_down) {
        armed = false;
    }

    //printf("was_down=%d\n", was_down);

    if (is_down && !was_down) {
        armed = true;
        for (int k=0; k<1000; ++k) {
            printf("ARM!!!\n");
        }
    }


    was_down = is_down;
    printf("armed=%d\n", armed);

    delay(1);
}
