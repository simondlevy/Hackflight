/*
   Support for MPU6050 IMU

   Adapted from https://github.com/kriswiner/MPU6050

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Wire.h>

namespace hf {

    class MPU6050 : public rft::Sensor {

        friend class HackflightFull;

        public:

            typedef enum {

                AFS_2G = 0,
                AFS_4G,
                AFS_8G,
                AFS_16G

            } ascale_t;

            typedef enum {

                GFS_250DPS = 0,
                GFS_500DPS,
                GFS_1000DPS,
                GFS_2000DPS

            } gscale_t;

            MPU6050(ascale_t ascale, gscale_t gscale)
            {
                _ascale = ascale;
                _gscale = gscale;

                _aRes = (1 << ((uint8_t)ascale + 1)) / 32768.0;
                _gRes = (250 * ((uint8_t)ascale + 1)) / 32768.0;
            }

        private:

            static const uint8_t  MPU6050_ADDRESS = 0x68;  

            static const uint8_t  SMPLRT_DIV     = 0x19;
            static const uint8_t  CONFIG         = 0x1A;
            static const uint8_t  GYRO_CONFIG    = 0x1B;
            static const uint8_t  ACCEL_CONFIG   = 0x1C;
            static const uint8_t  INT_PIN_CFG    = 0x37;
            static const uint8_t  INT_ENABLE     = 0x38;
            static const uint8_t  DMP_INT_STATUS = 0x39; 
            static const uint8_t  INT_STATUS     = 0x3A;
            static const uint8_t  ACCEL_XOUT_H   = 0x3B;
            static const uint8_t  GYRO_XOUT_H    = 0x43;
            static const uint8_t  PWR_MGMT_1     = 0x6B;

            // gyroscope measurement error in rads/s (start at 60 deg/s), then
            // reduce after ~10 s to 3
            static constexpr float GYRO_ERROR = PI * (40.0f / 180.0f);     

            // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            static constexpr float GYRO_DRIFT = PI * (2.0f / 180.0f);      

            // beta parameter for Madgwick quaternion filter
            static constexpr float BETA = sqrt(3.0f / 4.0f) * GYRO_ERROR;  

            // zeta parameter for Madgwick quaternion filter
            static constexpr float ZETA = sqrt(3.0f / 4.0f) * GYRO_DRIFT;  

            ascale_t _ascale;
            gscale_t _gscale;

            float _aRes = 0;
            float _gRes = 0;

            uint32_t _usec = 0;

            rft::MadgwickQuaternionFilter6DOF madgwick =
                rft::MadgwickQuaternionFilter6DOF(BETA, ZETA);

            void readData(float &x, float &y, float &z, uint8_t rgstr, float res)
            {
                uint8_t rawData[6]; 
                readBytes(MPU6050_ADDRESS, rgstr, 6, &rawData[0]); 
                x = (int16_t)((rawData[0] << 8) | rawData[1])  * res;
                y = (int16_t)((rawData[2] << 8) | rawData[3])  * res;
                z = (int16_t)((rawData[4] << 8) | rawData[5])  * res;
            }

            void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
            {
                Wire.beginTransmission(address);  
                Wire.write(subAddress);           
                Wire.write(data);                
                Wire.endTransmission();         
            }

            uint8_t readByte(uint8_t address, uint8_t subAddress)
            {
                uint8_t data; 
                readBytes(address, subAddress, 1, &data);
                return data;                          
            }

            void readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                    uint8_t * dest)
            {
                Wire.beginTransmission(address);   
                Wire.write(subAddress);            
                Wire.endTransmission(false);       
                uint8_t i = 0;
                Wire.requestFrom(address, count);  
                while (Wire.available()) {
                    dest[i++] = Wire.read();
                }         
            }

        protected:


            void begin()
            {
                // Wake up device
                writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);

                // Delay 100 ms for PLL to get established on x-axis gyro;
                // should check for PLL ready interrupt
                delay(100); 

                // Get stable time source: set clock source to be PLL with
                // x-axis gyroscope reference, bits 2:0 = 001
                writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  

                // Configure Gyro and Accelerometer Disable FSYNC and set
                // accelerometer and gyro bandwidth to 44 and 42 Hz,
                // respectively; DLPF_CFG = bits 2:0 = 010; this sets the
                // sample rate at 1 kHz for both Maximum delay time is 4.9 ms
                // corresponding to just over 200 Hz sample rate
                writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

                // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
                // Use a 200 Hz rate; the same rate set in CONFIG above
                writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  

                // Set gyroscope full scale range
                // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values
                // are left-shifted into positions 4:3
                uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
                writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); 
                writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);
                writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | _gscale << 3);

                // Set accelerometer configuration
                c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
                writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); 
                writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);
                writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | _ascale << 3); 

                // Configure Interrupts and Bypass Enable Set interrupt pin
                // active high, push-pull, and clear on read of INT_STATUS,
                // enable I2C_BYPASS_EN so additional chips can join the I2C
                // bus and all can be controlled by the Arduino as master
                writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
                writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);

                // Initialize clock for Madgwick filter
                _usec = 0;
            }

            virtual void modifyState(rft::State * state, float time)
            {
                State * hfstate = (State *)state;

                (void)time;

                // Check data ready
                if (!(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)) {
                    return;
                }

                // Get accelerometer
                float ax = 0, ay = 0, az = 0;
                readData(ax, ay, az, ACCEL_XOUT_H, _aRes);

                // Get gyrometer
                float gx = 0, gy = 0, gz = 0;
                readData(gx, gy, gz, GYRO_XOUT_H, _gRes);

                // Get current time difference for Madgwick filter
                uint32_t usec = micros();
                float deltat = (usec - _usec) / 1000000.0f;
                _usec = usec;

                // Use Madgwick filter to fuse accelerometer and gyrometer into quaternion
                madgwick.update(ax, ay, az, gx * PI / 180.0f,
                        gy * PI / 180.0f, gz * PI / 180.0f, deltat);

                // Convert quaternion to Euler angles
                Filter::quat2euler(
                        madgwick.q1,
                        madgwick.q2,
                        madgwick.q3,
                        madgwick.q4, 
                        hfstate->x[State::PHI],
                        hfstate->x[State::THETA],
                        hfstate->x[State::PSI]);

                // Adjust rotation so that nose-up is positive
                hfstate->x[State::THETA] = -hfstate->x[State::THETA];

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (hfstate->x[State::PSI] < 0) {
                    hfstate->x[State::PSI] += 2*M_PI;
                }

            } // modifyState

    }; // class MPU6050

} // namespace hf
