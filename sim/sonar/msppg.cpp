// AUTO-GENERATED CODE: DO NOT EDIT!!!


#include "msppg.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static byte CRC8(byte * data, int n) {

    byte crc = 0x00;

    for (int k=0; k<n; ++k) {

        crc ^= data[k];
    }

    return crc;
}

byte MSP_Message::start() {

    this->pos = 0;
    return this->getNext();
}

bool MSP_Message::hasNext() {

    return this->pos <= this->len;
}


byte MSP_Message::getNext() {

    return this->bytes[this->pos++];
}

MSP_Parser::MSP_Parser() {

    this->state = 0;
}

void MSP_Parser::parse(byte b) {

    switch (this->state) {

        case 0:               // sync char 1
            if (b == 36) { // $
                this->state++;
            }
            break;        

        case 1:               // sync char 2
            if (b == 77) { // M
                this->state++;
            }
            else {            // restart and try again
                this->state = 0;
            }
            break;

        case 2:               // direction (should be >)
            if (b == 62) { // >
                this->message_direction = 1;
            }
            else {            // <
                this->message_direction = 0;
            }
            this->state++;
            break;

        case 3:
            this->message_length_expected = b;
            this->message_checksum = b;
            // setup arraybuffer
            this->message_length_received = 0;
            this->state++;
            break;

        case 4:
            this->message_id = b;
            this->message_checksum ^= b;
            if (this->message_length_expected > 0) {
                // process payload
                this->state++;
            }
            else {
                // no payload
                this->state += 2;
            }
            break;

        case 5: // payload
            this->message_buffer[this->message_length_received] = b;
            this->message_checksum ^= b;
            this->message_length_received++;
            if (this->message_length_received >= this->message_length_expected) {
                this->state++;
            }
            break;

        case 6:
            this->state = 0;
            if (this->message_checksum == b) {
                // message received, process
                switch (this->message_id) {
                
                    case 108: {

                        short angx;
                        memcpy(&angx,  &this->message_buffer[0], sizeof(short));

                        short angy;
                        memcpy(&angy,  &this->message_buffer[2], sizeof(short));

                        short heading;
                        memcpy(&heading,  &this->message_buffer[4], sizeof(short));

                        this->handlerForATTITUDE->handle_ATTITUDE(angx, angy, heading);
                        } break;

                    case 105: {

                        short c1;
                        memcpy(&c1,  &this->message_buffer[0], sizeof(short));

                        short c2;
                        memcpy(&c2,  &this->message_buffer[2], sizeof(short));

                        short c3;
                        memcpy(&c3,  &this->message_buffer[4], sizeof(short));

                        short c4;
                        memcpy(&c4,  &this->message_buffer[6], sizeof(short));

                        short c5;
                        memcpy(&c5,  &this->message_buffer[8], sizeof(short));

                        short c6;
                        memcpy(&c6,  &this->message_buffer[10], sizeof(short));

                        short c7;
                        memcpy(&c7,  &this->message_buffer[12], sizeof(short));

                        short c8;
                        memcpy(&c8,  &this->message_buffer[14], sizeof(short));

                        this->handlerForRC->handle_RC(c1, c2, c3, c4, c5, c6, c7, c8);
                        } break;

                    case 109: {

                        int altitude;
                        memcpy(&altitude,  &this->message_buffer[0], sizeof(int));

                        short vario;
                        memcpy(&vario,  &this->message_buffer[4], sizeof(short));

                        this->handlerForALTITUDE->handle_ALTITUDE(altitude, vario);
                        } break;

                    case 127: {

                        short x;
                        memcpy(&x,  &this->message_buffer[0], sizeof(short));

                        short y;
                        memcpy(&y,  &this->message_buffer[2], sizeof(short));

                        short z;
                        memcpy(&z,  &this->message_buffer[4], sizeof(short));

                        short theta;
                        memcpy(&theta,  &this->message_buffer[6], sizeof(short));

                        this->handlerForSLAM_POSE->handle_SLAM_POSE(x, y, z, theta);
                        } break;

                }
            }

            break;

        default:
            break;
    }
}

MSP_Message MSP_Parser::serialize_SET_HEAD(short head) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 2;
    msg.bytes[4] = 205;

    memcpy(&msg.bytes[5], &head, sizeof(short));

    msg.bytes[7] = CRC8(&msg.bytes[3], 4);

    msg.len = 8;

    return msg;
}

void MSP_Parser::set_ATTITUDE_Handler(class ATTITUDE_Handler * handler) {

    this->handlerForATTITUDE = handler;
}

MSP_Message MSP_Parser::serialize_ATTITUDE_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 108;
    msg.bytes[5] = 108;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_ATTITUDE(short angx, short angy, short heading) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 6;
    msg.bytes[4] = 108;

    memcpy(&msg.bytes[5], &angx, sizeof(short));
    memcpy(&msg.bytes[7], &angy, sizeof(short));
    memcpy(&msg.bytes[9], &heading, sizeof(short));

    msg.bytes[11] = CRC8(&msg.bytes[3], 8);

    msg.len = 12;

    return msg;
}

void MSP_Parser::set_RC_Handler(class RC_Handler * handler) {

    this->handlerForRC = handler;
}

MSP_Message MSP_Parser::serialize_RC_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 105;
    msg.bytes[5] = 105;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 105;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 200;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

void MSP_Parser::set_ALTITUDE_Handler(class ALTITUDE_Handler * handler) {

    this->handlerForALTITUDE = handler;
}

MSP_Message MSP_Parser::serialize_ALTITUDE_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 109;
    msg.bytes[5] = 109;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_ALTITUDE(int altitude, short vario) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 6;
    msg.bytes[4] = 109;

    memcpy(&msg.bytes[5], &altitude, sizeof(int));
    memcpy(&msg.bytes[9], &vario, sizeof(short));

    msg.bytes[11] = CRC8(&msg.bytes[3], 8);

    msg.len = 12;

    return msg;
}

void MSP_Parser::set_SLAM_POSE_Handler(class SLAM_POSE_Handler * handler) {

    this->handlerForSLAM_POSE = handler;
}

MSP_Message MSP_Parser::serialize_SLAM_POSE_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 127;
    msg.bytes[5] = 127;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_SLAM_POSE(short x, short y, short z, short theta) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 8;
    msg.bytes[4] = 127;

    memcpy(&msg.bytes[5], &x, sizeof(short));
    memcpy(&msg.bytes[7], &y, sizeof(short));
    memcpy(&msg.bytes[9], &z, sizeof(short));
    memcpy(&msg.bytes[11], &theta, sizeof(short));

    msg.bytes[13] = CRC8(&msg.bytes[3], 10);

    msg.len = 14;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_MOTOR(short m1, short m2, short m3, short m4) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 8;
    msg.bytes[4] = 214;

    memcpy(&msg.bytes[5], &m1, sizeof(short));
    memcpy(&msg.bytes[7], &m2, sizeof(short));
    memcpy(&msg.bytes[9], &m3, sizeof(short));
    memcpy(&msg.bytes[11], &m4, sizeof(short));

    msg.bytes[13] = CRC8(&msg.bytes[3], 10);

    msg.len = 14;

    return msg;
}

