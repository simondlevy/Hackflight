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

#include <CRSFforArduino.hpp>

namespace hf {

    class RX {

        private:

            static constexpr uint32_t ELRS_TIMEOUT_MSEC = 500;

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

            uint32_t _last_rx_msec;

            CRSFforArduino _crsf;

            void scalechan(const uint8_t j, const uint8_t k)
            {
                chanvals[j] = map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
            }

            static void onReceiveRcChannels(
                    serialReceiverLayer::rcChannels_t *rcChannels, void * obj)
            {
                if (!rcChannels->failsafe) {

                    auto rx = (RX *)obj;

                    rx->scalechan(0 ,3);
                    rx->scalechan(1 ,1);
                    rx->scalechan(2 ,2);
                    rx->scalechan(3 ,4);
                    rx->scalechan(4 ,5);

                    rx->_last_rx_msec = millis();
                }
            }

        public:

            float chanvals[5];
            bool is_armed;
            bool is_throttle_down;

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

                const auto throttle_is_down = chanvals[0] < THROTTLE_DOWN_MAX;

                const auto msec_curr = millis();

                // Check failsafe via RX timeout
                if (_last_rx_msec > 0 &&
                        msec_curr > _last_rx_msec &&
                        msec_curr - _last_rx_msec > ELRS_TIMEOUT_MSEC) {
                    is_armed = false;
                }

                // Push-button arming
                static float _chan5_prev;
                const auto chan5_curr = chanvals[4];
                if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
                    is_armed = is_armed ? false : throttle_is_down ? true : is_armed;
                }
                _chan5_prev = chan5_curr;
            }
    };
}
