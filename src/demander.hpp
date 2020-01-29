/*
   Abstract "Demander" class for mixers and receiver proxies

   Copyright (c) 2020 Simon D. Levy

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

#include "datatypes.hpp"

namespace hf {

    class Demander {

        friend class Hackflight;
        friend class PidTask;

        protected:

            virtual void cut(void) = 0;

            virtual void run(demands_t demands) = 0;

    }; // class Demander

} // namespace hf
