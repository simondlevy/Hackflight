/*
Example for testing Python output of MSPPG

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

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

import edu.wlu.cs.msppg.*;

public class example implements ATTITUDE_Handler {

    public void handle_ATTITUDE(short angx, short angy, short heading) {

         System.out.printf("%+3d %+3d %+3d\n", angx, angy, heading);
    }

    public static void main(String [] argv) {

        Parser parser = new Parser();

        byte [] buf = parser.serialize_ATTITUDE((short)59, (short)76, (short)1);

        example handler = new example();

        parser.set_ATTITUDE_Handler(handler);

          for (byte b : buf) {

            parser.parse(b);

        }
    }

}
