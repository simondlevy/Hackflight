/*
   MSP parsing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2021

   MIT License
 */


#pragma once

#include <stdint.h>
#include <string.h>

namespace hf {

    class Parser {

        public:

            static const uint8_t MAXMSG = 255;

        private:

            static const int INBUF_SIZE  = 128;
            static const int OUTBUF_SIZE = 128;

            typedef enum serialState_t {
                IDLE,
                HEADER_START,
                HEADER_M,
                HEADER_ARROW,
                HEADER_SIZE,
                HEADER_CMD
            } serialState_t;

            uint8_t _checksum;
            uint8_t _inBuf[INBUF_SIZE];
            uint8_t _inBufIndex;
            uint8_t _outBuf[OUTBUF_SIZE];
            uint8_t _outBufIndex;
            uint8_t _outBufSize;
            uint8_t _command;
            uint8_t _offset;
            uint8_t _dataSize;
            uint8_t _direction;

            serialState_t  _state;

            void serialize8(uint8_t a)
            {
                _outBuf[_outBufSize++] = a;
                _checksum ^= a;
            }

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

            void headSerialResponse(uint8_t err, uint8_t s)
            {
                serialize8('$');
                serialize8('M');
                serialize8(err ? '!' : '>');
                _checksum = 0;               // start calculating a new _checksum
                serialize8(s);
                serialize8(_command);
            }

            void headSerialReply(uint8_t s)
            {
                headSerialResponse(0, s);
            }

            void prepareToSend(uint8_t count, uint8_t size)
            {
                _outBufSize = 0;
                _outBufIndex = 0;
                headSerialReply(count*size);
            }

            void prepareToSendBytes(uint8_t count)
            {
                prepareToSend(count, 1);
            }

            void sendByte(uint8_t src)
            {
                serialize8(src);
            }

            void prepareToSendShorts(uint8_t count)
            {
                prepareToSend(count, 2);
            }

            void sendShort(short src)
            {
                int16_t a;
                memcpy(&a, &src, 2);
                serialize16(a);
            }

            void prepareToSendInts(uint8_t count)
            {
                prepareToSend(count, 4);
            }

            void sendInt(int32_t src)
            {
                int32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void prepareToSendFloats(uint8_t count)
            {
                prepareToSend(count, 4);
            }

            void sendFloat(float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            static uint8_t CRC8(uint8_t * data, int n) 
            {
                uint8_t crc = 0x00;

                for (int k=0; k<n; ++k) {

                    crc ^= data[k];
                }

                return crc;
            }

        protected:

            void begin(void)
            {
                _checksum = 0;
                _outBufIndex = 0;
                _outBufSize = 0;
                _command = 0;
                _offset = 0;
                _dataSize = 0;
                _state = IDLE;
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

            // returns true if reboot request, false otherwise
            bool parse(uint8_t c)
            {
                switch (_state) {

                    case IDLE:
                        if (c == 'R') {
                            return true; // got reboot command
                        }
                        _state = (c == '$') ? HEADER_START : IDLE;
                        break;

                    case HEADER_START:
                        _state = (c == 'M') ? HEADER_M : IDLE;
                        break;

                    case HEADER_M:
                        switch (c) {
                           case '>':
                                _direction = 1;
                                _state = HEADER_ARROW;
                                break;
                            case '<':
                                _direction = 0;
                                _state = HEADER_ARROW;
                                break;
                             default:
                                _state = IDLE;
                        }
                        break;

                    case HEADER_ARROW:
                        if (c > INBUF_SIZE) {       // now we are expecting the payload size
                            _state = IDLE;
                            return false;
                        }
                        _dataSize = c;
                        _offset = 0;
                        _checksum = 0;
                        _inBufIndex = 0;
                        _checksum ^= c;
                        _state = HEADER_SIZE;      // the command is to follow
                        break;

                    case HEADER_SIZE:
                        _command = c;
                        _checksum ^= c;
                        _state = HEADER_CMD;
                        break;

                    case HEADER_CMD:
                        if (_offset < _dataSize) {
                            _checksum ^= c;
                            _inBuf[_offset++] = c;
                        } else  {
                            if (_checksum == c) {        // compare calculated and transferred _checksum
                                dispatchMessage();
                            }
                            _state = IDLE;
                        }

                } // switch (_state)

                return false; // no reboot 

            } // parse


            void dispatchMessage(void)
            {
                switch (_command) {

                    case 121:
                    {
                        float c1 = 0;
                        float c2 = 0;
                        float c3 = 0;
                        float c4 = 0;
                        float c5 = 0;
                        float c6 = 0;
                        handle_Receiver_Request(c1, c2, c3, c4, c5, c6);
                        prepareToSendFloats(6);
                        sendFloat(c1);
                        sendFloat(c2);
                        sendFloat(c3);
                        sendFloat(c4);
                        sendFloat(c5);
                        sendFloat(c6);
                        serialize8(_checksum);
                        } break;

                    case 122:
                    {
                        float roll = 0;
                        float pitch = 0;
                        float yaw = 0;
                        handle_ATTITUDE_RADIANS_Request(roll, pitch, yaw);
                        prepareToSendFloats(3);
                        sendFloat(roll);
                        sendFloat(pitch);
                        sendFloat(yaw);
                        serialize8(_checksum);
                        } break;

                    case 213:
                    {
                        float vx = 0;
                        memcpy(&vx,  &_inBuf[0], sizeof(float));

                        float vy = 0;
                        memcpy(&vy,  &_inBuf[4], sizeof(float));

                        float vz = 0;
                        memcpy(&vz,  &_inBuf[8], sizeof(float));

                        float yaw_rate = 0;
                        memcpy(&yaw_rate,  &_inBuf[12], sizeof(float));

                        handle_SET_VELOCITY_SETPOINTS(vx, vy, vz, yaw_rate);
                        } break;

                    case 215:
                    {
                        float m1 = 0;
                        memcpy(&m1,  &_inBuf[0], sizeof(float));

                        float m2 = 0;
                        memcpy(&m2,  &_inBuf[4], sizeof(float));

                        float m3 = 0;
                        memcpy(&m3,  &_inBuf[8], sizeof(float));

                        float m4 = 0;
                        memcpy(&m4,  &_inBuf[12], sizeof(float));

                        handle_SET_MOTOR_NORMAL(m1, m2, m3, m4);
                        } break;

                    case 217:
                    {
                        float c1 = 0;
                        memcpy(&c1,  &_inBuf[0], sizeof(float));

                        float c2 = 0;
                        memcpy(&c2,  &_inBuf[4], sizeof(float));

                        float c3 = 0;
                        memcpy(&c3,  &_inBuf[8], sizeof(float));

                        float c4 = 0;
                        memcpy(&c4,  &_inBuf[12], sizeof(float));

                        float c5 = 0;
                        memcpy(&c5,  &_inBuf[16], sizeof(float));

                        float c6 = 0;
                        memcpy(&c6,  &_inBuf[20], sizeof(float));

                        handle_SET_Receiver(c1, c2, c3, c4, c5, c6);
                        } break;

                }
            }

            virtual void handle_Receiver_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
            {
                (void)c1;
                (void)c2;
                (void)c3;
                (void)c4;
                (void)c5;
                (void)c6;
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw)
            {
                (void)roll;
                (void)pitch;
                (void)yaw;
            }

            virtual void handle_SET_VELOCITY_SETPOINTS(float  vx, float  vy, float  vz, float  yaw_rate)
            {
                (void)vx;
                (void)vy;
                (void)vz;
                (void)yaw_rate;
            }

            virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
            {
                (void)m1;
                (void)m2;
                (void)m3;
                (void)m4;
            }

            virtual void handle_SET_Receiver(float  c1, float  c2, float  c3, float  c4, float  c5, float  c6)
            {
                (void)c1;
                (void)c2;
                (void)c3;
                (void)c4;
                (void)c5;
                (void)c6;
            }

    }; // class Parser

} // namespace hf
