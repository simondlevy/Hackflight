/*
   sim.hpp : Support USB controller for flight simulators

   Controller subclasses Receiver

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "receiver.hpp"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#include <board.hpp>

namespace hf {

    class Controller : public Receiver {

        public:

            bool useSerial(void)
            {
                return true;
            }

            Controller(void)
            {
                for (int k=0; k<6; ++k)
                    _axisdir[k] = +1;

                _joyfd = 0;
            }

            void begin(void)
            {
               getProduct(_vendorId, _productId);
                _throttleDemand = -1.f;
            }

            uint16_t readChannel(uint8_t chan)
            {
                // Poll on first channel request
                if (chan == 0) {
                    poll();
                }

                // Special handling for throttle
                float demand = (chan == 0) ? _throttleDemand : _demands[chan];

                // Joystick demands are in [-1,+1]; convert to [1000,2000]
                return (uint16_t)(demand*500 + 1500);
            }

            void halt(void)
            {
            }

        private:

            static const uint16_t VENDOR_SONY      = 0x0e8f;
            static const uint16_t VENDOR_STM       = 0x0483;
            static const uint16_t VENDOR_LOGITECH  = 0x046d;
            static const uint16_t VENDOR_MICROSOFT = 0x24c6;
            static const uint16_t PRODUCT_TARANIS  = 0x5710;

            // implemented differently for each OS
            void getProduct(uint16_t & vendorId, uint16_t & productId);
            void pollProduct(void);

            void poll(void)
            {
                pollProduct();

                // game-controller spring-mounted throttle requires special handling
                switch (_vendorId) {
                    case VENDOR_SONY:
                    case VENDOR_MICROSOFT:
                        if (abs(_demands[0]) < .15) {
                            _demands[0] = 0; // deadband filter
                        }
                        _throttleDemand += _demands[0] * .01f; // XXX need to make this deltaT computable
                        if (_throttleDemand < -1)
                            _throttleDemand = -1;
                        if (_throttleDemand > 1)
                            _throttleDemand = 1;
                        break;
                    default:
                        _throttleDemand = _demands[0];
                }
            }

            static float joynorm(int axisval) 
            {
                return (axisval - (float)32767) / 32767;
            }

            uint16_t     _vendorId;
            uint16_t     _productId;
            float        _throttleDemand;
            float        _demands[8];
            uint8_t      _axismap[8];
            int8_t       _axisdir[8];
            int          _joyfd; // needed for Linux only

    }; // class Controller

} // namespace hf
