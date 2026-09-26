#include <hackflight.h>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class NewReceiver {

        public:

            NewReceiver(
                    const MspParser & parser,
                    const bool is_down,
                    const bool was_down,
                    const bool armed) 
                : parser_(parser),
                is_down_(is_down),
                was_down_(was_down),
                is_armed_(armed) {}

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

                return NewReceiver(parser, is_down, rx.was_down_, rx.is_armed_);
            }

            static auto Update(const NewReceiver & rx) -> NewReceiver
            {
                const auto armed = !rx.is_down_ ? false : rx.is_down_ && !rx.was_down_ ? true : rx.is_armed_;

                return NewReceiver(rx.parser_, rx.is_down_, rx.is_down_, armed);
            }

            static auto IsArmed(const NewReceiver & rx) -> bool
            {
                return rx.is_armed_;
            }

        private:

            MspParser parser_;
            bool is_down_;
            bool was_down_;
            bool is_armed_;
    };
}
