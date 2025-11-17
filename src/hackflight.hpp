/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <stdint.h>

#define _MAIN

#include <debugger.hpp>
#include <ekf.hpp>
#include <tasks/core.hpp>
#include <tasks/hover.hpp>

class Hackflight {

    public:

        void init(const uint8_t motorCount, const mixFun_t mixFun)
        {
            _ekf.init(millis());

            _hoverTask.begin(&_ekf);

            _coreTask.begin(&_ekf, motorCount, mixFun);
        }

    private:

        CoreTask _coreTask;
        HoverTask _hoverTask;
        EKF _ekf;
        Debugger _debugger;
};
