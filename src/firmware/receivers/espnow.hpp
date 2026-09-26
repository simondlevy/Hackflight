#pragma once

#if 0

            static auto Update(
                    const EspNowReceiver & rx,
                    const uint8_t byte,
                    const uint32_t time_msec
                    ) -> EspNowReceiver
            {
                const bool is_arming_button_up = !GetSwitchStatus(rx.parser_, 4);

                const bool is_armed  =

                    // Disarm when arming button is up
                    is_arming_button_up ? false :

                    // Arm when arming button goes up to down and throttle is down
                    (!rx.is_armed_ &&
                    GetThrottle(rx) < kThrottleDownMax && 
                    !is_arming_button_up &&
                    rx.was_arming_button_up_) ? true :

                    // Otherwise leave arming status alone
                    rx.is_armed_;

                printf("%d\n", is_armed);

                return EspNowReceiver(
                        MspParser::Parse(rx.parser_, byte),
                        MspParser::GetId(rx.parser_) == kMspSetChannels ?  time_msec :
                        rx.time_msec_,
                        is_arming_button_up,
                        is_armed);
            }

            static auto DidRequestHover(const EspNowReceiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 5);
            }

            static auto DidRequestAutopilot(const EspNowReceiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 6);
            }

            static auto GetTimestampMsec(const EspNowReceiver & rx) -> uint32_t
            {
                return rx.time_msec_;
            }
    };
}
#endif
