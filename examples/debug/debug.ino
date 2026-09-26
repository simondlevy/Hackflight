#include <hackflight.h>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class NewReceiver {

        public:

            MspParser parser_;
            bool is_down_;
            bool was_down_;
            bool armed_;

            NewReceiver(
                    const MspParser & parser,
                    const bool is_down,
                    const bool was_down,
                    const bool armed) 
                : parser_(parser),
                is_down_(is_down),
                was_down_(was_down),
                armed_(armed) {}

            NewReceiver() : was_down_(true) {}

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

                return NewReceiver(parser, is_down, rx.was_down_, rx.armed_);
            }

            static auto Update(const NewReceiver & rx) -> NewReceiver
            {
                const auto armed = !rx.is_down_ ? false : rx.is_down_ && !rx.was_down_ ? true : rx.armed_;

                return NewReceiver(rx.parser_, rx.is_down_, rx.is_down_, armed);
            }

            void Update()
            {
                armed_ = !is_down_ ? false : is_down_ && !was_down_ ? true : armed_;
                was_down_ = is_down_;

            }
    };
}

static hf::NewReceiver rx_;

void serialEvent3()
{
    while (Serial3.available()) {

        rx_ = hf::NewReceiver::ParseByte(rx_, Serial3.read(), millis());
    }
}

void setup()
{
    Serial3.begin(115200);

    delay(3000);
}

void loop()
{
    //rx_.Update();

    rx_ = hf::NewReceiver::Update(rx_);

    printf("armed=%d\n", rx_.armed_);

    delay(1);
}
