/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <CRSFforArduino.hpp>

// Hackflight
#include <hackflight.h>
#include <firmware/datatypes.hpp>

namespace hf {

    class RX {

        private:

            static constexpr uint32_t ELRS_TIMEOUT_MSEC = 500;

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

            float scalechan(const uint8_t k)
            {
                return map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
            }

            static void onReceiveRcChannels(
                    serialReceiverLayer::rcChannels_t *rcChannels, void * obj)
            {
                if (!rcChannels->failsafe) {

                    auto rx = (RX *)obj;

                    rx->data.axes.thrust = rx->scalechan(3);
                    rx->data.axes.roll = rx->scalechan(1);
                    rx->data.axes.pitch = rx->scalechan(2);
                    rx->data.axes.yaw = rx->scalechan(4);

                    rx->data.aux = rx->scalechan(5);

                    rx->_last_rx_msec = millis();
                }
            }

        public:

            RxData data;

            void begin()
            {
                if (!_crsf.begin()) {

                    _crsf.end();

                    while (true) {
                        printf("CRSF for Arduino initialisation failed!\n");
                        delay(500);
                    }
                }

                _crsf.setRcChannelsCallback(onReceiveRcChannels, this);
            }

            void read()
            {
                _crsf.update();

                data.is_throttle_down = data.axes.thrust < THROTTLE_DOWN_MAX;

                const auto msec_curr = millis();

                // Check failsafe via RX timeout
                if (_last_rx_msec > 0 &&
                        msec_curr > _last_rx_msec &&
                        msec_curr - _last_rx_msec > ELRS_TIMEOUT_MSEC) {
                    data.is_armed = false;
                }

                // Push-button arming
                static float _chan5_prev;
                const auto chan5_curr = data.aux;
                if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
                    data.is_armed =
                        data.is_armed ? false :
                        data.is_throttle_down ? true :
                        data.is_armed;
                }
                _chan5_prev = chan5_curr;
            }

        private:

            uint32_t _last_rx_msec;

            CRSFforArduino _crsf;

    };
}
