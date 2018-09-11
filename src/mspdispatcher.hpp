/*
   mspdispatcher.hpp : abstract class for classes than dispatch MSP messages

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

namespace hf {

    class MspDispatcher {

        friend class Hackflight;
        friend class MspParser;

        protected:

        // See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
        typedef enum {

            CMD_NONE                 =   0,
            CMD_GET_RC_NORMAL        = 121,
            CMD_GET_ATTITUDE_RADIANS = 122, 
            CMD_GET_ALTITUDE_METERS  = 123, 
            CMD_SET_MOTOR_NORMAL     = 215,    
            CMD_SET_ARMED            = 216

        } Command_t;

        virtual void dispatchMspCommand(Command_t command) = 0;

    }; // class MspDispatcher

} // namespace hf
