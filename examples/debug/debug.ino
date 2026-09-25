#include <hackflight.h>
#include <firmware/debugger.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

void serialEvent3()
{
    while (Serial3.available()) {

        static hf::MspParser parser_;

        parser_ = hf::MspParser::Parse(parser_, Serial3.read());

        if (hf::MspParser::GetId(parser_) == kMspSetChannels) {

            static bool was_down;

            const auto is_down = hf::MspParser::GetShort(parser_, 4) > 0;

            if (is_down && !was_down) {
                printf("ARM!!!\n");
            }

            was_down = is_down;
        }
    }
}

void setup()
{
    Serial3.begin(115200);

    delay(3000);
}

void loop()
{
}
