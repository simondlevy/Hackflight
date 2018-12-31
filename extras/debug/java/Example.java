/*
   Example for testing Python output of MSPPG

   Requires comm.jar:

   https://www.oracle.com/technetwork/java/javasebusiness/downloads/java-archive-downloads-misc-419423.html

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

import edu.wlu.cs.msppg.*;
import java.io.IOException;
import javax.comm.*;


public class Example implements ATTITUDE_RADIANS_Handler {

    public void handle_ATTITUDE_RADIANS(float angx, float angy, float heading) {

        System.out.printf("%+3.3f %+3.3f %+3.3f\n", angx, angy, heading);
    }

    public static void main(String [] argv) {

        Parser parser = new Parser();

        byte [] buf = parser.serialize_ATTITUDE_RADIANS_Request();

        try {

            System.out.write(buf);
        }

        catch (IOException e) {
        }


        while (true) {

            try {
                int inChar = System.in.read();
                System.out.println(inChar);
            }
            catch (IOException e){
                break;
            }
        }
        /*
           Example handler = new Example();

           parser.set_ATTITUDE_RADIANS_Handler(handler);

           for (byte b : buf) {

           parser.parse(b);

           }*/
    }

}
