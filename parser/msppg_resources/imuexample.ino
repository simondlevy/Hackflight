/*
Example for testing Arduino output of MSPPG.  Reports IMU values coming in
on Serial1 and uses pitch to modify the pitch of a buzzer connetected on pin 8.

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

#include <msppg.h>

MSP_Parser parser;

MSP_Message request = parser.serialize_ATTITUDE_Request();

class My_ATTITUDE_Handler : public ATTITUDE_Handler {


    public:

        void handle_ATTITUDE(short angx, short angy, short heading) {

            // Report the attitude
            Serial.print("pitch: " );
            Serial.print(angy/10.);
            Serial.print(" | roll: " );
            Serial.print(angx/10.);
            Serial.print(" | yaw: " );
            Serial.println(heading);

            // Use pitch to set the pitch!
            tone(8, map(angy, -1800, 1800, 100, 1000));

            // Start the process over again
            this->sendRequest();
        }

        void sendRequest() {

          for (byte b=request.start(); request.hasNext(); b=request.getNext()) {

              Serial1.write(b);
          }  
        }
};

My_ATTITUDE_Handler handler;

void setup() {

    Serial.begin(115200); 

    Serial1.begin(115200); 

    parser.set_ATTITUDE_Handler(&handler);

    handler.sendRequest();
}

void loop() {

    if (Serial1.available()) {

        parser.parse(Serial1.read());
    }
}
