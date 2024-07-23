#pragma once

class Timer {

    public:

        bool isReady(const uint32_t usec_curr, const float freq_hz)
        {
            const auto is_ready = (usec_curr - _usec_prev) > (1e6 / freq_hz);
            
            _usec_prev = is_ready ? usec_curr : _usec_prev;

            return is_ready;
        }

    private:

        uint32_t _usec_prev;
};
