/*
   Serial comms with parser

   MIT License
 */

#pragma once

#include "../copilot_extra.h"

#include "mixer.hpp"
#include "mixer.hpp"
#include "state.hpp"
#include "receiver.hpp"

namespace hf {

    class SerialTask {

        friend class Hackflight;

        protected:

            void parse(Mixer * mixer, State * vehicleState, serial_t & serial)
            {
                enum {
                    IDLE,
                    GOT_START,
                    GOT_M,
                    GOT_ARROW,
                    GOT_SIZE,
                    IN_PAYLOAD
                }; 

                static uint8_t parser_state;

                static uint8_t type;
                static uint8_t crc;
                static uint8_t size;
                static uint8_t index;

                uint8_t c = copilot_serialByte;

                // Payload functions
                size = parser_state == GOT_ARROW ? c : size;
                index = parser_state == IN_PAYLOAD ? index + 1 : 0;
                bool incoming = type >= 200;
                bool in_payload = incoming && parser_state == IN_PAYLOAD;

                // Command acquisition function
                type = parser_state == GOT_SIZE ? c : type;

                // Checksum transition function
                crc = parser_state == GOT_ARROW ? c
                    : parser_state == IN_PAYLOAD  ?  crc ^ c 
                    : 0;

                // Parser state transition function
                parser_state
                    = parser_state == IDLE && c == '$' ? GOT_START
                    : parser_state == GOT_START && c == 'M' ? GOT_M
                    : parser_state == GOT_M && (c == '<' || c == '>') ? GOT_ARROW
                    : parser_state == GOT_ARROW ? GOT_SIZE
                    : parser_state == GOT_SIZE ? IN_PAYLOAD
                    : parser_state == IN_PAYLOAD && index < size ? IN_PAYLOAD
                    : parser_state == IN_PAYLOAD ? IDLE
                    : parser_state;

                serial.count = 0; // assume nothing

                // Payload accumulation
                if (in_payload) {
                    serial.count = -1; // incoming
                    serial.input = c;
                }

                // Message dispatch
                if (parser_state == IDLE && crc == c) {

                    if (size > 0) {
                        handleSerialInput(mixer, type);
                    }

                    else {

                        prepareSerialOutput(vehicleState, mixer, type, serial);
                    }
                }

            } // parse

            void prepareSerialOutput(State * vehicleState, Mixer * mixer, uint8_t type, serial_t & serial)
            {
                serial.type = type;

                switch (type) {

                    case 121:
                        {
                            serial.count = 6;
                            serial.output01 = copilot_receiverThrottle;
                            serial.output02 = copilot_receiverRoll;
                            serial.output03 = copilot_receiverPitch;
                            serial.output04 = copilot_receiverYaw;
                            serial.output05 = copilot_receiverAux1;
                            serial.output06 = 0; // XXX we should support aux2
                        } break;

                    case 122:
                        {
                            serial.count = 12;
                            float * x = vehicleState->x;
                            serial.output01 = x[State::X];
                            serial.output02 = x[State::DX];
                            serial.output03 = x[State::Y];
                            serial.output04 = x[State::DY];
                            serial.output05 = x[State::Z];
                            serial.output06 = x[State::DZ];
                            serial.output07 = x[State::PHI];
                            serial.output08 = x[State::DPHI];
                            serial.output09 = x[State::THETA];
                            serial.output10 = x[State::DTHETA];
                            serial.output11 = x[State::PSI];
                            serial.output12 = x[State::DPSI];
                        } break;

                    case 123:
                        {
                            serial.count = 1;
                            serial.output01 = mixer->getType();
                        } break;

                } // switch (type)

            } // prepareSerialOutput

            void handleSerialInput(Mixer * mixer, uint8_t type)
            {
                switch (type) {

                    case 215:
                        {
                            mixer->setMotorDisarmed(0, copilot_Input1);
                            mixer->setMotorDisarmed(1, copilot_Input2);
                            mixer->setMotorDisarmed(2, copilot_Input3);
                            mixer->setMotorDisarmed(3, copilot_Input4);

                        } break;

                } // switch (type)

            } // handleSerialInput

    }; // class SerialTask

} // namespace hf
