/*
   Datatypes for firmware

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

#include <datatypes.hpp>

namespace hf {

    class ReceiverData {

        public:

            Setpoint setpoint;
            bool requested_arming;
            bool requested_hover;
            uint32_t timestamp_msec;

            ReceiverData() = default;

            ReceiverData(const ReceiverData & r) 
                : setpoint(r.setpoint),
                requested_arming(r.requested_arming),
                requested_hover(r.requested_hover),
                timestamp_msec(r.timestamp_msec) {}

            ReceiverData(
                    const Setpoint & setpoint,
                    const bool requested_arming,
                    const bool requested_hover,
                    const uint32_t timestamp_msec)
                : setpoint(setpoint),
                requested_arming(requested_arming),
                requested_hover(requested_hover),
                timestamp_msec(timestamp_msec) {}

    }; // class ReceiverData

} // namespace hf
