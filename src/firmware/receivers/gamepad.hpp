/*
   Receiver class for gamepad control

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

#include <firmware/datatypes.hpp>

namespace hf {

    class GamepadReceiver {

        public:

            ReceiverData data;

            void HandleSerial1Event()
            {
                while (Serial1.available()) {

                    HandleIncomingByte(Serial1.read());
                }
            }

        private:

            void HandleIncomingByte(const uint8_t byte)
            {
                static MspParser _parser;

                _parser = MspParser::Parse(_parser, byte);

                switch (MspParser::GetId(_parser)) {

                    case kMspSetArming:
                        data.requested_arming = !data.requested_arming;
                        data.timestamp_msec = millis();
                        break;

                    case kMspSetIdle:
                        data.requested_hover = false;
                        data.timestamp_msec = millis();
                        break;

                    case kMspSetHover:
                        data.requested_hover = true;
                        data.setpoint.thrust =
                            MspParser::GetFloat(_parser, 0);
                        data.setpoint.pitch =
                            MspParser::GetFloat(_parser, 1); // vx
                        data.setpoint.roll =
                            MspParser::GetFloat(_parser, 2); // vy
                        data.setpoint.yaw =
                            MspParser::GetFloat(_parser, 3);
                        data.timestamp_msec = millis();
                        break;

                    default:
                        break;
                }
            }

    }; // class GamepadReceiver

} // namespace hf
