/*
   esp8266.hpp : ESP8266 support for Arduino flight controllers

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

#include<ESP8266WiFi.h>

namespace hf {

    class ESP8266_Receiver : public Receiver {

         protected:

            void begin(void)
            {
            }

            bool gotNewFrame(void)
            {
                return false; //rx.gotNewFrame();
            }

            void readRawvals(void)
            {
                //rx.getChannelValuesNormalized(rawvals, CHANNELS);
            }

            bool lostSignal(void)
            {
                return false; //rx.timedOut(micros());
            }

        public:

            ESP8266_Receiver(const uint8_t channelMap[6]) : Receiver(channelMap) 
            { 
            }

    }; // class ESP8266_Receiver

} // namespace

/*

// WiFi Definitions
static const char* ssid = "Esp8266TestNet";
const static char* password = "Esp8266Test"; // has to be longer than 7 chars

WiFiServer server(80);

void setup() {

    Serial.begin(115200);
    delay(10);

    WiFi.mode(WIFI_AP);
    //WiFi.softAP(ssid, password, 1, 1);
    WiFi.softAP(ssid);

    server.begin();
}

void loop() {

    static WiFiClient client;
    static bool haveClient;

    if (haveClient) {

        if (client.connected()) {

            if (client.available()) {
                Serial.println(client.readStringUntil('\r'));
            }
        }

        else {
            haveClient = false;
        }
    }

    else {
        client = server.available();
        if (client) {
            haveClient = true;
        } 
        else {
            Serial.println("Waiting for client ...");
        }
    }

    delay(500);

    //String request = client.readStringUntil('\r');
    //Serial.println(request);
    //client.flush();
}

*/
