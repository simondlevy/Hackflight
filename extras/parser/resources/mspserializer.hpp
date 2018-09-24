/*
   mspserializer.hpp: header-only implementation of MSP serializing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2018

   This program is part of Hackflight

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as 
   published by the Free Software Foundation, either version 3 of the 
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,     
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License 
   along with this code.  If not, see <http:#www.gnu.org/licenses/>.
 */


#pragma once

#include <stdint.h>

namespace hf {

    class MspMessage {

        friend class MspParser;

        protected:

        static const int MAXBUF = 256;

        uint8_t _bytes[MAXBUF];
        int _pos;
        int _len;

        public:

        uint8_t start(void) 
        {
            _pos = 0;
            return getNext();
        }

        bool hasNext(void) 
        {
            return _pos <= _len;
        }


        uint8_t getNext(void) 
        {
            return _bytes[_pos++];
        }

    };

    class MspSerializer {

