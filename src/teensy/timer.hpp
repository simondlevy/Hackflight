/*
   Timing tasks support for Hackflight

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

class Timer {

    public:

        Timer(const float freq_hz)
        {
            _freq_hz = freq_hz;
        }

        bool isReady(const uint32_t usec_curr)
        {
            const auto is_ready = (usec_curr - _usec_prev) > (1e6 / _freq_hz);

            _usec_prev = is_ready ? usec_curr : _usec_prev;

            return is_ready;
        }

    private:

        uint32_t _usec_prev;

        float _freq_hz;
};
