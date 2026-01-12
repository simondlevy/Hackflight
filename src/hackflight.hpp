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

#include <comms.hpp>
#include <tasks/task1.hpp>
#include <tasks/task2.hpp>

#include <newekf.hpp>

class Hackflight {

    public:

        void init()
        {
            Comms::init();

            _ekf.init(millis());

            _task1.begin(&_ekf);

            _task2.begin(&_ekf);
        }

    private:

        NewEKF _ekf;
        Task1 _task1;
        Task2 _task2;
};
