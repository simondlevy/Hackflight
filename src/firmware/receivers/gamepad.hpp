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

            void handleSerial1Event()
            {
                while (Serial1.available()) {

                    handleIncomingByte(Serial1.read());
                }
            }

        private:

            void handleIncomingByte(const uint8_t byte)
            {
                static MspParser _parser;

                _parser = MspParser::parse(_parser, byte);

                switch (MspParser::getid(_parser)) {

                    case MSP_SET_ARMING:
                        data.requested_arming = !data.requested_arming;
                        data.timestamp_msec = millis();
                        break;

                    case MSP_SET_IDLE:
                        data.requested_hover = false;
                        data.timestamp_msec = millis();
                        break;

                    case MSP_SET_HOVER:
                        data.requested_hover = true;
                        data.setpoint.thrust =
                            MspParser::getFloat(_parser, 0);
                        data.setpoint.pitch =
                            MspParser::getFloat(_parser, 1); // vx
                        data.setpoint.roll =
                            MspParser::getFloat(_parser, 2); // vy
                        data.setpoint.yaw =
                            MspParser::getFloat(_parser, 3);
                        data.timestamp_msec = millis();
                        break;

                    default:
                        break;
                }
            }

    }; // class GamepadReceiver

} // namespace hf
