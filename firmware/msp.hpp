/*
   msp.hpp : MSP (Multiwii Serial Protocol) class header

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

#define CONFIG_REBOOT_CHARACTER 'R'

#ifdef __arm__
extern "C" {
#endif

    static const int INBUF_SIZE = 128;

    typedef enum serialState_t {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD
    } serialState_t;

    typedef  struct mspPortState_t {
        uint8_t checksum;
        uint8_t indRX;
        uint8_t inBuf[INBUF_SIZE];
        uint8_t cmdMSP;
        uint8_t offset;
        uint8_t dataSize;
        serialState_t c_state;
    } mspPortState_t;

    class MSP {

        private:

            Board * _board;
            IMU * _imu;
            Mixer * _mixer;
            RC * _rc;

            mspPortState_t portState;

            void serialize8(uint8_t a);
            void serialize16(int16_t a);
            uint8_t read8(void);
            uint16_t read16(void);
            uint32_t read32(void);
            void serialize32(uint32_t a);
            void headSerialResponse(uint8_t err, uint8_t s);
            void headSerialReply(uint8_t s);
            void headSerialError(uint8_t s);
            void tailSerialReply(void);

        public:

            void init(Board * board, IMU * imu, Mixer * mixer, RC * rc);

            void update(bool armed);

    }; // class MSP

#ifdef __arm__
} // extern "C"
#endif
