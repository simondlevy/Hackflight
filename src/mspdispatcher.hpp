/*
   mspdispatcher.hpp : dispatch current MSP commands

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "mspparser.hpp"

namespace hf {

    class MspDispatcher : public MspParser {

        protected:

        static const uint8_t CMD_GET_RC_NORMAL        = 121;
        static const uint8_t CMD_GET_ATTITUDE_RADIANS = 122;
        static const uint8_t CMD_GET_ALTITUDE_METERS  = 123;
        static const uint8_t CMD_GET_LOITER_RAW       = 126;
        static const uint8_t CMD_SET_MOTOR_NORMAL     = 215;
        static const uint8_t CMD_SET_ARMED            = 216;

        virtual void dispatchCommand(uint8_t cmd) override
        {
            switch (cmd) {

                case MspParser::CMD_SET_MOTOR_NORMAL:
                    dispatch_SET_MOTOR_NORMAL();
                    break;

                case MspParser::CMD_SET_ARMED:
                    dispatch_SET_ARMED();
                    break;

                case MspParser::CMD_GET_RC_NORMAL:
                    dispatch_GET_RC_NORMAL();
                    break;

                case MspParser::CMD_GET_ATTITUDE_RADIANS: 
                    dispatch_GET_ATTITUDE_RADIANS();
                    break;

                case MspParser::CMD_GET_ALTITUDE_METERS: 
                    dispatch_GET_ALTITUDE_METERS();
                    break;

                    // don't know how to handle the (valid) message, indicate error
                default:           
                    MspParser::error();        
                    break;
            }
        }

        virtual void dispatch_SET_MOTOR_NORMAL() { }
        virtual void dispatch_SET_ARMED() { }
        virtual void dispatch_GET_RC_NORMAL() { }
        virtual void dispatch_GET_ATTITUDE_RADIANS() { }
        virtual void dispatch_GET_ALTITUDE_METERS() { }

    }; // cassMspDispatcher

} // namespace hf
