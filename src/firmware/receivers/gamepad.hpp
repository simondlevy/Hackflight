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


namespace hf {

    class GamepadReceiver {

        public:

            uint32_t timestamp_msec;
            bool requested_arming;
            bool requested_hover;
            Setpoint setpoint;

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
                        requested_arming =
                            !requested_arming;
                        timestamp_msec = millis();
                        break;

                    case MSP_SET_IDLE:
                        requested_hover = false;
                        timestamp_msec = millis();
                        break;

                    case MSP_SET_HOVER:
                        requested_hover = true;
                        setpoint.thrust =
                            MspParser::getFloat(_parser, 0);
                        setpoint.pitch =
                            MspParser::getFloat(_parser, 1); // vx
                        setpoint.roll =
                            MspParser::getFloat(_parser, 2); // vy
                        setpoint.yaw =
                            MspParser::getFloat(_parser, 3);
                        timestamp_msec = millis();
                        break;

                    default:
                        break;
                }
            }

    }; // class GamepadReceiver

} // namespace hf
