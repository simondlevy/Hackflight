/*
   MSP parsing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2021

   MIT License
 */


#pragma once

#include <stdint.h>
#include <string.h>

#include <RFT_parser.hpp>

namespace hf {

    class Parser : public rft::Parser {

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
