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

#include <hackflight.h>
#include <firmware/device/timer.hpp>

namespace hf {

    class Profiler {

        public:

            void report()
            {
                if (_timer.ready()) {

                    if (_count > 0) {
                        printf("count=%d\n", (int)_count);
                    }

                    _count = 0;
                }
                _count++;
            }

        private:

            Timer _timer = Timer(1); // 1 Hz reporting

            uint32_t _count;
    };


}
