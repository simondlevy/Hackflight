#pragma once

#include <timer.hpp>
#include <datatypes.h>
#include <ekf.hpp>

class EkfPredictTask {

    public:

        void run( const uint32_t usec_curr, const float freq_hz, Ekf & ekf)
        {
            if (_timer.isReady(usec_curr, freq_hz)) {

                ekf.predict(usec_curr/1000);
            }
        }

    private:

        Timer _timer;
};
