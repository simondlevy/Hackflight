/*
   Serial comms with parser

   MIT License
 */

/*
#pragma once

#include "mixer.hpp"
#include "mixer.hpp"
#include "state.hpp"
#include "receiver.hpp"

#include "../Debugger.hpp"

extern Debugger debugger;

namespace hf {

    typedef struct {

        float v00;
        float v01;
        float v02;
        float v03;
        float v04;
        float v05;
        float v06;
        float v07;
        float v08;
        float v09;
        float v10;
        float v11;

    } output_t;

    typedef struct {

        uint32_t w00;
        uint32_t w01;
        uint32_t w02;
        uint32_t w03;

    } input_t;

    typedef struct {

        int8_t count; // 0=nothing; -1=incoming; +=outgoing
        uint8_t type;
        input_t input;
        bool input_ready;
        output_t output;

    } serial_t;

    class SerialTask {

        friend class Hackflight;

        private:

        void prepareSerialOutput(
                State * vehicleState,
                Mixer * mixer,
                uint8_t type,
                serial_t & serial)
        {
            switch (type) {

                case 121:
                    {
                        serial.count = 6;
                        serial.output.v00 = copilot_receiverThrottle;
                        serial.output.v01 = copilot_receiverRoll;
                        serial.output.v02 = copilot_receiverPitch;
                        serial.output.v03 = copilot_receiverYaw;
                        serial.output.v04 = copilot_receiverAux1;
                        serial.output.v05 = 0; // XXX we should support aux2
                    } break;

                case 122:
                    {
                        serial.count = 12;
                        float * x = vehicleState->x;
                        serial.output.v00 = x[State::X];
                        serial.output.v01 = x[State::DX];
                        serial.output.v02 = x[State::Y];
                        serial.output.v03 = x[State::DY];
                        serial.output.v04 = x[State::Z];
                        serial.output.v05 = x[State::DZ];
                        serial.output.v06 = x[State::PHI];
                        serial.output.v07 = x[State::DPHI];
                        serial.output.v08 = x[State::THETA];
                        serial.output.v09 = x[State::DTHETA];
                        serial.output.v10 = x[State::PSI];
                        serial.output.v11 = x[State::DPSI];
                    } break;

            } // switch (type)

        } // prepareSerialOutput

        void handleSerialInput(Mixer * mixer, uint8_t type)
        {
            switch (type) {

                case 215:
                    {
                        motors[0] = copilot_input1;
                        motors[1] = copilot_input2;
                        motors[2] = copilot_input3;
                        motors[3] = copilot_input4;

                    } break;

            } // switch (type)

        } // handleSerialInput

        static uint8_t mtype2count(uint8_t mt) 
        {
            return mt == 121 ? 6
                : mt == 122 ? 12
                : 0;
        }

        void set_input(input_t * input, bool in_payload, uint8_t index, uint8_t b)
        {
            if (in_payload) {
                switch (index) {
                    case 0:  input->w00  = (uint32_t)b; break;
                    case 1:  input->w00 |= (uint32_t)b << 8; break;
                    case 2:  input->w00 |= (uint32_t)b << 16; break;
                    case 3:  input->w00 |= (uint32_t)b << 24; break;
                    case 4:  input->w01 =  (uint32_t)b; break;
                    case 5:  input->w01 |= (uint32_t)b << 8; break;
                    case 6:  input->w01 |= (uint32_t)b << 16; break;
                    case 7:  input->w01 |= (uint32_t)b << 16; break;
                    case 8:  input->w02 =  (uint32_t)b; break;
                    case 9:  input->w02 |= (uint32_t)b << 8; break;
                    case 10: input->w02 |= (uint32_t)b << 16; break;
                    case 11: input->w02 |= (uint32_t)b << 24; break;
                    case 12: input->w03 =  (uint32_t)b; break;
                    case 13: input->w03 |= (uint32_t)b << 8; break;
                    case 14: input->w03 |= (uint32_t)b << 16; break;
                    case 15: input->w03 |= (uint32_t)b << 24; break;
                }
            }
        }

        protected:

        float motors[4] = {};

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

            serial.input_ready = parser_state == IN_PAYLOAD && index == size;

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

            bool ready = parser_state == IDLE && crc == c;

            serial.count = in_payload ? -1
                : ready && size > 0 ? mtype2count(type)
                : 0;

            set_input(&serial.input, in_payload, index-1, c);

            // Message dispatch
            if (ready) {

                serial.type = type;

                if (size > 0) {
                    handleSerialInput(mixer, type);
                }

                else {

                    prepareSerialOutput(vehicleState, mixer, type, serial);
                }
            }

        } // parse

    }; // class SerialTask

} // namespace hf
*/
