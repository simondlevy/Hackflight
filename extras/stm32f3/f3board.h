/*
   f3board.h : Support for STM32F3 boards

   Copyright (C) 2018 Simon D. Levy 

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

#include <boards/realboard.hpp>

class F3Board : public hf::RealBoard {

    friend class hf::Board;

    protected:

        void delaySeconds(float sec);

        void ledSet(bool is_on);

        virtual uint32_t getMicroseconds(void) override;

        virtual void reboot(void) override;

        static void outchar(char c);

    public:

        F3Board(void);

}; // class F3Board
