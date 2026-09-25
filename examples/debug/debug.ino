#include <hackflight.h>
#include <firmware/debugger.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

static hf::MspParser parser_;

void serialEvent3()
{
    while (Serial3.available()) {

        parser_ = hf::MspParser::Parse(parser_, Serial3.read());

        if (hf::MspParser::GetId(parser_) == kMspSetChannels) {

                printf("%d\n", hf::MspParser::GetShort(parser_, 4) > 0);
        }
    }
}

void setup()
{
    Serial3.begin(115200);
}

void loop()
{

    delay(10);
}
