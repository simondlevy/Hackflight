/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <VL53L1X.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>

namespace hf {

    class ZRanger {

        private:

            static constexpr auto DISTANCE_MODE = VL53L1X::Medium;
            static constexpr uint32_t TIMING_BUDGET_USEC = 25'000;

        public:

            void begin(TwoWire & wire=Wire1)
            {
                wire.begin();
                wire.setClock(400000);

                _vl53l1x.setBus(&wire);

                delay(100);

                if (!_vl53l1x.init()) {
                    Debugger::reportForever("VL53L1X initialization failed");
                }

                _vl53l1x.setDistanceMode(VL53L1X::Medium);
                _vl53l1x.setMeasurementTimingBudget(25000);
                _vl53l1x.startContinuous(50);

            }

            auto available() -> bool
            {
                return _vl53l1x.dataReady();
            }

            auto read() -> uint16_t
            {
                return _vl53l1x.read();
            }

        private:

            VL53L1X _vl53l1x;
    };
}
