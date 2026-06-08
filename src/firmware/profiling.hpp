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

#include <hackflight.h>
#include <firmware/timer.hpp>

namespace hf {

    class Profiler {

        public:

            void report()
            {
                if (timer_.Ready()) {

                    if (count_ > 0) {
                        printf("count=%d\n", (int)count_);
                    }

                    count_ = 0;
                }
                count_++;
            }

        private:

            Timer timer_ = Timer(1); // 1 Hz reporting

            uint32_t count_;
    };


}
