/*
   serial.hpp : Class declaration for SerialConnection class

   Copyright (C) Simon D. Levy 2016

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

class SerialConnection {

    public:

        SerialConnection(const char * portname, int baudrate=9600, bool blocking=true, int parity=0);

        bool openConnection(void);

        int bytesAvailable(void);

        int readBytes(char * buf, int size);

        int writeBytes(char * buf, int size);

        void closeConnection(void);

    private:

        int fd;
        char portname[100];
        int baudrate;
        bool blocking;
        int parity;

};
