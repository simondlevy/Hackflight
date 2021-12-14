/*
   MSP parsing routines

   Copyright (C) Simon D. Levy 2021

   MIT License
 */


#pragma once

#include <stdint.h>
#include <string.h>

class Parser {

    private:

        static const uint8_t MAXMSG = 255;

        static const int OUTBUF_SIZE = 128;

        uint8_t _outBufChecksum;
        uint8_t _outBuf[OUTBUF_SIZE];
        uint8_t _outBufIndex;
        uint8_t _outBufSize;

        void serialize16(int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        void serialize32(uint32_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
            serialize8((a >> 16) & 0xFF);
            serialize8((a >> 24) & 0xFF);
        }

        void prepareToSend(uint8_t type, uint8_t count, uint8_t size)
        {
            _outBufSize = 0;
            _outBufIndex = 0;
            _outBufChecksum = 0;

            addToOutBuf('$');
            addToOutBuf('M');
            addToOutBuf('>');
            serialize8(count*size);
            serialize8(type);
        }

        void addToOutBuf(uint8_t a)
        {
            _outBuf[_outBufSize++] = a;
        }

    protected:

        void completeSend(void)
        {
            serialize8(_outBufChecksum);
        }

        void serialize8(uint8_t a)
        {
            addToOutBuf(a);
            _outBufChecksum ^= a;
        }

        void prepareToSendBytes(uint8_t type, uint8_t count)
        {
            prepareToSend(type, count, 1);
        }

        void sendByte(uint8_t src)
        {
            serialize8(src);
        }

        void prepareToSendShorts(uint8_t type, uint8_t count)
        {
            prepareToSend(type, count, 2);
        }

        void sendShort(short src)
        {
            int16_t a;
            memcpy(&a, &src, 2);
            serialize16(a);
        }

        void prepareToSendInts(uint8_t type, uint8_t count)
        {
            prepareToSend(type, count, 4);
        }

        void sendInt(int32_t src)
        {
                int32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void prepareToSendFloats(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 4);
            }

            void sendFloat(float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            virtual void collectPayload(uint8_t index, uint8_t value) = 0;
            virtual void dispatchMessage(uint8_t type) = 0;

    public:

            void begin(void)
            {
                _outBufChecksum = 0;
                _outBufIndex = 0;
                _outBufSize = 0;
            }

            uint8_t availableBytes(void)
            {
                return _outBufSize;
            }

            uint8_t readByte(void)
            {
                _outBufSize--;
                return _outBuf[_outBufIndex++];
            }

            void parse(uint8_t c)
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

                printf("x%02X: %d\n", c, parser_state);

                // Payload accumulation
                if (in_payload) {
                    collectPayload(index-1, c);
                }

                // Message dispatch
                if (parser_state == IDLE && crc == c) {
                    dispatchMessage(type);
                }

            } // parse

    }; // class Parser
