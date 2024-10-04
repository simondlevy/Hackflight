/*
   Hackflight communications task

   Copyright (C) 2024 Simon D. Levy

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

#include <hackflight.hpp>
#include <timer.hpp>

namespace hf {

    class CommsTask {

        public:

            void run(const uint32_t usec_curr, const float freq_hz)
            {
                if (_timer.isReady(usec_curr, freq_hz)) {

                    static uint8_t count;

                    Serial3.write(count);

                    count = (count + 1) % 100;
                }
            }

        private:

            Timer _timer;
    };

}
