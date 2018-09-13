// AUTO-GENERATED CODE: DO NOT EDIT!!!


#include "MSPPG.h"

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
                
                    case 126: {

                        byte agl;
                        memcpy(&agl,  &this->message_buffer[0], sizeof(byte));

                        byte flowx;
                        memcpy(&flowx,  &this->message_buffer[1], sizeof(byte));

                        byte flowy;
                        memcpy(&flowy,  &this->message_buffer[2], sizeof(byte));

                        this->handlerForGET_LOITER_RAW->handle_GET_LOITER_RAW(agl, flowx, flowy);
                        } break;

                    case 122: {

                        float roll;
                        memcpy(&roll,  &this->message_buffer[0], sizeof(float));

                        float pitch;
                        memcpy(&pitch,  &this->message_buffer[4], sizeof(float));

                        float yaw;
                        memcpy(&yaw,  &this->message_buffer[8], sizeof(float));

                        this->handlerForGET_ATTITUDE_RADIANS->handle_GET_ATTITUDE_RADIANS(roll, pitch, yaw);
                        } break;

                    case 123: {

                        float estalt;
                        memcpy(&estalt,  &this->message_buffer[0], sizeof(float));

                        float vario;
                        memcpy(&vario,  &this->message_buffer[4], sizeof(float));

                        this->handlerForGET_ALTITUDE_METERS->handle_GET_ALTITUDE_METERS(estalt, vario);
                        } break;

                    case 121: {

                        float c1;
                        memcpy(&c1,  &this->message_buffer[0], sizeof(float));

                        float c2;
                        memcpy(&c2,  &this->message_buffer[4], sizeof(float));

                        float c3;
                        memcpy(&c3,  &this->message_buffer[8], sizeof(float));

                        float c4;
                        memcpy(&c4,  &this->message_buffer[12], sizeof(float));

                        float c5;
                        memcpy(&c5,  &this->message_buffer[16], sizeof(float));

                        float c6;
                        memcpy(&c6,  &this->message_buffer[20], sizeof(float));

                        this->handlerForGET_RC_NORMAL->handle_GET_RC_NORMAL(c1, c2, c3, c4, c5, c6);
                        } break;

                }
            }

            break;

        default:
            break;
    }
}

void MSP_Parser::set_GET_LOITER_RAW_Handler(class GET_LOITER_RAW_Handler * handler) {

    this->handlerForGET_LOITER_RAW = handler;
}

MSP_Message MSP_Parser::serialize_GET_LOITER_RAW_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 126;
    msg.bytes[5] = 126;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_GET_LOITER_RAW(byte agl, byte flowx, byte flowy) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 3;
    msg.bytes[4] = 126;

    memcpy(&msg.bytes[5], &agl, sizeof(byte));
    memcpy(&msg.bytes[6], &flowx, sizeof(byte));
    memcpy(&msg.bytes[7], &flowy, sizeof(byte));

    msg.bytes[8] = CRC8(&msg.bytes[3], 5);

    msg.len = 9;

    return msg;
}

void MSP_Parser::set_GET_ATTITUDE_RADIANS_Handler(class GET_ATTITUDE_RADIANS_Handler * handler) {

    this->handlerForGET_ATTITUDE_RADIANS = handler;
}

MSP_Message MSP_Parser::serialize_GET_ATTITUDE_RADIANS_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 122;
    msg.bytes[5] = 122;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_GET_ATTITUDE_RADIANS(float roll, float pitch, float yaw) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 12;
    msg.bytes[4] = 122;

    memcpy(&msg.bytes[5], &roll, sizeof(float));
    memcpy(&msg.bytes[9], &pitch, sizeof(float));
    memcpy(&msg.bytes[13], &yaw, sizeof(float));

    msg.bytes[17] = CRC8(&msg.bytes[3], 14);

    msg.len = 18;

    return msg;
}

void MSP_Parser::set_GET_ALTITUDE_METERS_Handler(class GET_ALTITUDE_METERS_Handler * handler) {

    this->handlerForGET_ALTITUDE_METERS = handler;
}

MSP_Message MSP_Parser::serialize_GET_ALTITUDE_METERS_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 123;
    msg.bytes[5] = 123;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_GET_ALTITUDE_METERS(float estalt, float vario) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 8;
    msg.bytes[4] = 123;

    memcpy(&msg.bytes[5], &estalt, sizeof(float));
    memcpy(&msg.bytes[9], &vario, sizeof(float));

    msg.bytes[13] = CRC8(&msg.bytes[3], 10);

    msg.len = 14;

    return msg;
}

void MSP_Parser::set_GET_RC_NORMAL_Handler(class GET_RC_NORMAL_Handler * handler) {

    this->handlerForGET_RC_NORMAL = handler;
}

MSP_Message MSP_Parser::serialize_GET_RC_NORMAL_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 121;
    msg.bytes[5] = 121;

    msg.len = 6;

    return msg;
}

MSP_Message MSP_Parser::serialize_GET_RC_NORMAL(float c1, float c2, float c3, float c4, float c5, float c6) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 24;
    msg.bytes[4] = 121;

    memcpy(&msg.bytes[5], &c1, sizeof(float));
    memcpy(&msg.bytes[9], &c2, sizeof(float));
    memcpy(&msg.bytes[13], &c3, sizeof(float));
    memcpy(&msg.bytes[17], &c4, sizeof(float));
    memcpy(&msg.bytes[21], &c5, sizeof(float));
    memcpy(&msg.bytes[25], &c6, sizeof(float));

    msg.bytes[29] = CRC8(&msg.bytes[3], 26);

    msg.len = 30;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_MOTOR_NORMAL(float m1, float m2, float m3, float m4) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 215;

    memcpy(&msg.bytes[5], &m1, sizeof(float));
    memcpy(&msg.bytes[9], &m2, sizeof(float));
    memcpy(&msg.bytes[13], &m3, sizeof(float));
    memcpy(&msg.bytes[17], &m4, sizeof(float));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_ARMED(byte flag) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 1;
    msg.bytes[4] = 216;

    memcpy(&msg.bytes[5], &flag, sizeof(byte));

    msg.bytes[6] = CRC8(&msg.bytes[3], 3);

    msg.len = 7;

    return msg;
}

