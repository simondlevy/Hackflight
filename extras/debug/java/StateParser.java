/*
   Example of parsing MSPPG messages with Java

   Mocks up an STATE message response from the flight controller
   and uses the Parser object to parse the response bytes

   Copyright (C) Simon D. Levy 2018

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

import edu.wlu.cs.msppg.Parser;

public class StateParser extends Parser {

    @Override
    public void handle_STATE(float  altitude,float variometer, float positionX, float positionY, 
            float heading, float velocityForward, float velocityRightward) {

        System.out.printf("%+3.3f %+3.3f %+3.3f %+3.3f %+3.3f %+3.3f %+3.3f\n", 
                altitude,variometer, positionX, positionY, 
                heading, velocityForward, velocityRightward);

    }

    public static void main(String [] argv) {

        // Fake response from flight controller
        byte [] buf = {(byte)0x24, (byte)0x4d, (byte)0x3e, (byte)0x1c,
            (byte)0x70, (byte)0x00, (byte)0x00, (byte)0x80, (byte)0x3f,
            (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x40, (byte)0x00,
            (byte)0x00, (byte)0x40, (byte)0x40, (byte)0x00, (byte)0x00,
            (byte)0x80, (byte)0x40, (byte)0x00, (byte)0x00, (byte)0xa0,
            (byte)0x40, (byte)0x00, (byte)0x00, (byte)0xc0, (byte)0x40,
            (byte)0x00, (byte)0x00, (byte)0xe0, (byte)0x40, (byte)0x93};

        StateParser parser = new StateParser();

        for (byte b : buf) {

            parser.parse(b);

        }
    }

}
