/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#pragma once

#include <motors_api.h>

class MotorsTask {

    public:

        int getRatio(uint32_t id)
        {
            return ratios[id];
        }

        void stop()
        {
            if (didInit) {
                motors_stop();
            }
        }

        void begin(void)
        {
            if (didInit) {
                return;
            }

            didInit = true;

            motors_init();

            stop();
        }

        bool test(void)
        {
            return didInit;
        }

        void setRatios(const uint16_t ratios[])
        {
            setRatio(0, ratios[0]);
            setRatio(1, ratios[1]);
            setRatio(2, ratios[2]);
            setRatio(3, ratios[3]);
        }

    private:

        bool didInit = false;

        uint32_t ratios[4]; 

        void setRatio(uint32_t id, uint16_t ratio)
        {
            ratios[id] = ratio;

            motors_setRatio(id, ratio);
        }

};
