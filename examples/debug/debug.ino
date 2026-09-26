#include <hackflight.h>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class NewReceiver {

        public:
            MspParser parser_;

            bool is_down_;

            NewReceiver(const MspParser & parser, const bool is_down) 
                : parser_(parser), is_down_(is_down) {}

            NewReceiver() = default;

            NewReceiver& operator=(const NewReceiver& other) = default;

            static auto ParseByte(
                    const NewReceiver & rx,
                    const uint8_t byte,
                    const uint32_t time_msec
                    ) -> NewReceiver
            {
                auto parser = MspParser::Parse(rx.parser_, byte);

                const auto is_down =
                    MspParser::GetId(parser) == kMspSetChannels ?
                    MspParser::GetShort(parser, 4) > 0 :
                    rx.is_down_;

                return NewReceiver(parser, is_down);
            }
    };
}

static hf::NewReceiver rx_;

static bool armed;
static bool was_down;

void serialEvent3()
{
    while (Serial3.available()) {

        rx_ = hf::NewReceiver::ParseByte(rx_, Serial3.read(), millis());
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
    armed = !rx_.is_down_ ? false : rx_.is_down_ && !was_down ? true : armed;

    was_down = rx_.is_down_;

    printf("armed=%d\n", armed);

    delay(1);
}
