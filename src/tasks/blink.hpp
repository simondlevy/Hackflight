#pragma once

#include <timer.hpp>

class BlinkTask {

    public:

        void run(const uint32_t usec_curr, const float freq_hz)
        {
            static uint32_t _usec_prev;

            static uint32_t _delay;

            if (usec_curr - _usec_prev > _delay) {

                static bool _alternate;

                _usec_prev = usec_curr;

                digitalWrite(LED_BUILTIN, _alternate);

                if (_alternate) {
                    _alternate = false;
                    _delay = UPTIME_USEC;
                }
                else {
                    _alternate = true;
                    _delay = freq_hz * 1e6;
                }
            }
        }

    private:

        static const uint32_t UPTIME_USEC = 100'000;
};
