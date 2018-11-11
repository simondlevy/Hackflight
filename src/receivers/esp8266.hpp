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
#include "mspparser.hpp"

namespace hf {

    class ESP8266_Receiver : public Receiver, public MspParser {

        private:

            char _ssid[100];
            char _passwd[100];

            WiFiServer _server = WiFiServer(80);
            WiFiClient _client;
            bool _haveClient;

        protected:

            void begin(void)
            {
                WiFi.mode(WIFI_AP);
                if (strlen(_passwd) > 0) {
                    WiFi.softAP(_ssid, _passwd, 1, 1);
                }
                else {
                    WiFi.softAP(_ssid); // no password
                }
                _server.begin();
                _haveClient = false;

                MspParser::init();
            }

            bool gotNewFrame(void)
            {
                if (_haveClient) {

                    if (_client.connected()) {

                        while (_client.available()) {
                            MspParser::parse(_client.read());
                        }
                    }

                    else {
                        _haveClient = false;
                    }
                }

                else {
                    _client = _server.available();
                    if (_client) {
                        //Serial.println("Connected!");
                        _haveClient = true;
                    } 
                    else {
                        //Serial.println("Waiting for client ...");
                    }
                }

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

            virtual void handle_SET_RC_NORMAL_Request(float  c1, float  c2, float  c3, float  c4, float  c5, float  c6) override
            {
                Debug::printf("%+2.2f %+2.2f %+2.2f %+2.2f %+2.2f %+2.2f\n", c1, c2, c3, c4, c5, c6);
            }

         public:

        ESP8266_Receiver(const uint8_t channelMap[6], const char * ssid, const char * passwd="") : Receiver(channelMap) 
        { 
            strcpy(_ssid, ssid);
            strcpy(_passwd, passwd);
        }

    }; // class ESP8266_Receiver

} // namespace hf
