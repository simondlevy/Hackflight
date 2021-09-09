#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>


class MPU6050
{
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
        }

        float getGres() {

            switch (_gscale)
            {
                // Possible gyro scales (and their register bit settings) are:
                // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
                // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
                case GFS_250DPS:
                    return 250.0 / 32768.0;
                    break;
                case GFS_500DPS:
                    return 500.0 / 32768.0;
                    break;
                case GFS_1000DPS:
                    return 1000.0 / 32768.0;
                    break;
                case GFS_2000DPS:
                    return 2000.0 / 32768.0;
                    break;
            }
        }


        float getAres() {
            switch (_ascale)
            {
                // Possible accelerometer scales (and their register bit settings) are:
                // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
                // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
                case AFS_2G:
                    return 2.0 / 32768.0;
                    break;
                case AFS_4G:
                    return 4.0 / 32768.0;
                    break;
                case AFS_8G:
                    return 8.0 / 32768.0;
                    break;
                case AFS_16G:
                    return 16.0 / 32768.0;
                    break;
            }
        }


        void readAccelData(int16_t * destination)
        {
            uint8_t rawData[6]; 
            readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); 
            destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;
            destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
            destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
        }

        void readGyroData(int16_t * destination)
        {
            uint8_t rawData[6];  
            readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
            destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;
            destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
            destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
        }



        void begin()
        {
            // wake up device-don't need this here if using calibration function below
            //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
            //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

            // get stable time source
            writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

            // Configure Gyro and Accelerometer
            // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
            // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
            // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
            writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

            // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
            writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

            // Set gyroscope full scale range
            // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
            uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
            writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
            writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
            writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | _gscale << 3); // Set full scale range for the gyro

            // Set accelerometer configuration
            c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
            writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
            writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
            writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | _ascale << 3); // Set full scale range for the accelerometer

            // Configure Interrupts and Bypass Enable
            // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
            // can join the I2C bus and all can be controlled by the Arduino as master
            writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
            writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
        }

        // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
        // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
        void calibrateMPU6050(float * dest1, float * dest2)
        {
            uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
            uint16_t ii, packet_count, fifo_count;
            int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

            // reset device, reset all registers, clear gyro and accelerometer bias registers
            writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
            delay(100);

            // get stable time source
            // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
            writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
            writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
            delay(200);

            // Configure device for bias calculation
            writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
            writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
            writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
            writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
            writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
            writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
            delay(15);

            // Configure MPU6050 gyro and accelerometer for bias calculation
            writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
            writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
            writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
            writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

            uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
            uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

            // Configure FIFO to capture accelerometer and gyro data for bias calculation
            writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
            writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
            delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

            // At end of sample accumulation, turn off FIFO sensor read
            writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
            readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
            fifo_count = ((uint16_t)data[0] << 8) | data[1];
            packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

            for (ii = 0; ii < packet_count; ii++) {
                int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
                readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
                accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
                accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
                accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
                gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
                gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
                gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

                accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
                accel_bias[1] += (int32_t) accel_temp[1];
                accel_bias[2] += (int32_t) accel_temp[2];
                gyro_bias[0]  += (int32_t) gyro_temp[0];
                gyro_bias[1]  += (int32_t) gyro_temp[1];
                gyro_bias[2]  += (int32_t) gyro_temp[2];

            }
            accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
            accel_bias[1] /= (int32_t) packet_count;
            accel_bias[2] /= (int32_t) packet_count;
            gyro_bias[0]  /= (int32_t) packet_count;
            gyro_bias[1]  /= (int32_t) packet_count;
            gyro_bias[2]  /= (int32_t) packet_count;

            if (accel_bias[2] > 0L) {
                accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
            }
            else {
                accel_bias[2] += (int32_t) accelsensitivity;
            }

            // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
            data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
            data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
            data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
            data[3] = (-gyro_bias[1] / 4)       & 0xFF;
            data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
            data[5] = (-gyro_bias[2] / 4)       & 0xFF;

            // Push gyro biases to hardware registers
            writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
            writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
            writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
            writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
            writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
            writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

            dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
            dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
            dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

            // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
            // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
            // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
            // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
            // the accelerometer biases calculated above must be divided by 8.

            int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
            readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
            accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
            readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
            accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
            readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
            accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

            uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
            uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

            for (ii = 0; ii < 3; ii++) {
                if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
            }

            // Construct total accelerometer bias, including calculated average accelerometer bias from above
            accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
            accel_bias_reg[1] -= (accel_bias[1] / 8);
            accel_bias_reg[2] -= (accel_bias[2] / 8);

            data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
            data[1] = (accel_bias_reg[0])      & 0xFF;
            data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
            data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
            data[3] = (accel_bias_reg[1])      & 0xFF;
            data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
            data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
            data[5] = (accel_bias_reg[2])      & 0xFF;
            data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

            // Push accelerometer biases to hardware registers
            writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
            writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
            writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
            writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
            writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
            writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

            // Output scaled accelerometer biases for manual subtraction in the main program
            dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
            dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
            dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
        }


        // Accelerometer and gyroscope self test; check calibration wrt factory settings
        void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
        {
            uint8_t rawData[4];
            uint8_t selfTest[6];
            float factoryTrim[6];

            // Configure the accelerometer for self-test
            writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
            writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
            delay(250);  // Delay a while to let the device execute the self-test
            rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
            rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
            rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
            rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
            // Extract the acceleration test results first
            selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
            selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
            selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
            // Extract the gyration test results first
            selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
            selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
            selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
            // Process results to allow final comparison with factory set values
            factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
            factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
            factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
            factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
            factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
            factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

            //  Output self-test results and factory trim calculation if desired
            //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
            //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
            //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
            //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

            // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
            // To get to percent, must multiply by 100 and subtract result from 100
            for (int i = 0; i < 6; i++) {
                destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
            }

        }

        bool dataReady(void)
        {
            return (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01);
        }
    private:

        static const uint8_t  MPU6050_ADDRESS     = 0x68;  // Device address when ADO   = 1;

        static const uint8_t  XGOFFS_TC           = 0x00; // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD;
        static const uint8_t  YGOFFS_TC           = 0x01;
        static const uint8_t  ZGOFFS_TC           = 0x02;
        static const uint8_t  X_FINE_GAIN         = 0x03; // [7:0] fine gain;
        static const uint8_t  Y_FINE_GAIN         = 0x04;
        static const uint8_t  Z_FINE_GAIN         = 0x05;
        static const uint8_t  XA_OFFSET_H         = 0x06; // User-defined trim values for accelerometer;
        static const uint8_t  XA_OFFSET_L_TC      = 0x07;
        static const uint8_t  YA_OFFSET_H         = 0x08;
        static const uint8_t  YA_OFFSET_L_TC      = 0x09;
        static const uint8_t  ZA_OFFSET_H         = 0x0A;
        static const uint8_t  ZA_OFFSET_L_TC      = 0x0B;
        static const uint8_t  SELF_TEST_X         = 0x0D;
        static const uint8_t  SELF_TEST_Y         = 0x0E;
        static const uint8_t  SELF_TEST_Z         = 0x0F;
        static const uint8_t  SELF_TEST_A         = 0x10;
        static const uint8_t  XG_OFFS_USRH        = 0x13;  // User-defined trim values for gyroscope; supported in MPU-6050?;
        static const uint8_t  XG_OFFS_USRL        = 0x14;
        static const uint8_t  YG_OFFS_USRH        = 0x15;
        static const uint8_t  YG_OFFS_USRL        = 0x16;
        static const uint8_t  ZG_OFFS_USRH        = 0x17;
        static const uint8_t  ZG_OFFS_USRL        = 0x18;
        static const uint8_t  SMPLRT_DIV          = 0x19;
        static const uint8_t  CONFIG              = 0x1A;
        static const uint8_t  GYRO_CONFIG         = 0x1B;
        static const uint8_t  ACCEL_CONFIG        = 0x1C;
        static const uint8_t  FF_THR              = 0x1D;  // Free-fall;
        static const uint8_t  FF_DUR              = 0x1E;  // Free-fall;
        static const uint8_t  MOT_THR             = 0x1F;  // Motion detection threshold bits [7:0];
        static const uint8_t  MOT_DUR             = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB   = 1 ms;
        static const uint8_t  ZMOT_THR            = 0x21;  // Zero-motion detection threshold bits [7:0];
        static const uint8_t  ZRMOT_DUR           = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB   = 64 ms;
        static const uint8_t  FIFO_EN             = 0x23;
        static const uint8_t  I2C_MST_CTRL        = 0x24;
        static const uint8_t  I2C_SLV0_ADDR       = 0x25;
        static const uint8_t  I2C_SLV0_REG        = 0x26;
        static const uint8_t  I2C_SLV0_CTRL       = 0x27;
        static const uint8_t  I2C_SLV1_ADDR       = 0x28;
        static const uint8_t  I2C_SLV1_REG        = 0x29;
        static const uint8_t  I2C_SLV1_CTRL       = 0x2A;
        static const uint8_t  I2C_SLV2_ADDR       = 0x2B;
        static const uint8_t  I2C_SLV2_REG        = 0x2C;
        static const uint8_t  I2C_SLV2_CTRL       = 0x2D;
        static const uint8_t  I2C_SLV3_ADDR       = 0x2E;
        static const uint8_t  I2C_SLV3_REG        = 0x2F;
        static const uint8_t  I2C_SLV3_CTRL       = 0x30;
        static const uint8_t  I2C_SLV4_ADDR       = 0x31;
        static const uint8_t  I2C_SLV4_REG        = 0x32;
        static const uint8_t  I2C_SLV4_DO         = 0x33;
        static const uint8_t  I2C_SLV4_CTRL       = 0x34;
        static const uint8_t  I2C_SLV4_DI         = 0x35;
        static const uint8_t  I2C_MST_STATUS      = 0x36;
        static const uint8_t  INT_PIN_CFG         = 0x37;
        static const uint8_t  INT_ENABLE          = 0x38;
        static const uint8_t  DMP_INT_STATUS      = 0x39;  // Check DMP interrupt;
        static const uint8_t  INT_STATUS          = 0x3A;
        static const uint8_t  ACCEL_XOUT_H        = 0x3B;
        static const uint8_t  ACCEL_XOUT_L        = 0x3C;
        static const uint8_t  ACCEL_YOUT_H        = 0x3D;
        static const uint8_t  ACCEL_YOUT_L        = 0x3E;
        static const uint8_t  ACCEL_ZOUT_H        = 0x3F;
        static const uint8_t  ACCEL_ZOUT_L        = 0x40;
        static const uint8_t  TEMP_OUT_H          = 0x41;
        static const uint8_t  TEMP_OUT_L          = 0x42;
        static const uint8_t  GYRO_XOUT_H         = 0x43;
        static const uint8_t  GYRO_XOUT_L         = 0x44;
        static const uint8_t  GYRO_YOUT_H         = 0x45;
        static const uint8_t  GYRO_YOUT_L         = 0x46;
        static const uint8_t  GYRO_ZOUT_H         = 0x47;
        static const uint8_t  GYRO_ZOUT_L         = 0x48;
        static const uint8_t  EXT_SENS_DATA_00    = 0x49;
        static const uint8_t  EXT_SENS_DATA_01    = 0x4A;
        static const uint8_t  EXT_SENS_DATA_02    = 0x4B;
        static const uint8_t  EXT_SENS_DATA_03    = 0x4C;
        static const uint8_t  EXT_SENS_DATA_04    = 0x4D;
        static const uint8_t  EXT_SENS_DATA_05    = 0x4E;
        static const uint8_t  EXT_SENS_DATA_06    = 0x4F;
        static const uint8_t  EXT_SENS_DATA_07    = 0x50;
        static const uint8_t  EXT_SENS_DATA_08    = 0x51;
        static const uint8_t  EXT_SENS_DATA_09    = 0x52;
        static const uint8_t  EXT_SENS_DATA_10    = 0x53;
        static const uint8_t  EXT_SENS_DATA_11    = 0x54;
        static const uint8_t  EXT_SENS_DATA_12    = 0x55;
        static const uint8_t  EXT_SENS_DATA_13    = 0x56;
        static const uint8_t  EXT_SENS_DATA_14    = 0x57;
        static const uint8_t  EXT_SENS_DATA_15    = 0x58;
        static const uint8_t  EXT_SENS_DATA_16    = 0x59;
        static const uint8_t  EXT_SENS_DATA_17    = 0x5A;
        static const uint8_t  EXT_SENS_DATA_18    = 0x5B;
        static const uint8_t  EXT_SENS_DATA_19    = 0x5C;
        static const uint8_t  EXT_SENS_DATA_20    = 0x5D;
        static const uint8_t  EXT_SENS_DATA_21    = 0x5E;
        static const uint8_t  EXT_SENS_DATA_22    = 0x5F;
        static const uint8_t  EXT_SENS_DATA_23    = 0x60;
        static const uint8_t  MOT_DETECT_STATUS   = 0x61;
        static const uint8_t  I2C_SLV0_DO         = 0x63;
        static const uint8_t  I2C_SLV1_DO         = 0x64;
        static const uint8_t  I2C_SLV2_DO         = 0x65;
        static const uint8_t  I2C_SLV3_DO         = 0x66;
        static const uint8_t  I2C_MST_DELAY_CTRL  = 0x67;
        static const uint8_t  SIGNAL_PATH_RESET   = 0x68;
        static const uint8_t  MOT_DETECT_CTRL     = 0x69;
        static const uint8_t  USER_CTRL           = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP;
        static const uint8_t  PWR_MGMT_1          = 0x6B; // Device defaults to the SLEEP mode;
        static const uint8_t  PWR_MGMT_2          = 0x6C;
        static const uint8_t  DMP_BANK            = 0x6D;  // Activates a specific bank in the DMP;
        static const uint8_t  DMP_RW_PNT          = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank;
        static const uint8_t  DMP_REG             = 0x6F;  // Register in DMP from which to read or to which to write;
        static const uint8_t  DMP_REG_1           = 0x70;
        static const uint8_t  DMP_REG_2           = 0x71;
        static const uint8_t  FIFO_COUNTH         = 0x72;
        static const uint8_t  FIFO_COUNTL         = 0x73;
        static const uint8_t  FIFO_R_W            = 0x74;
        static const uint8_t  WHO_AM_I_MPU6050    = 0x75; // Should return  = 0x68;

        ascale_t _ascale;
        gscale_t _gscale;



        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
        {
            Wire.beginTransmission(address);  // Initialize the Tx buffer
            Wire.write(subAddress);           // Put slave register address in Tx buffer
            Wire.write(data);                 // Put data in Tx buffer
            Wire.endTransmission();           // Send the Tx buffer
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress)
        {
            uint8_t data; // `data` will store the register data
            Wire.beginTransmission(address);         // Initialize the Tx buffer
            Wire.write(subAddress);	                 // Put slave register address in Tx buffer
            Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
            Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
            data = Wire.read();                      // Fill Rx buffer with result
            return data;                             // Return data read from slave register
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
        {
            Wire.beginTransmission(address);   // Initialize the Tx buffer
            Wire.write(subAddress);            // Put slave register address in Tx buffer
            Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
            uint8_t i = 0;
            Wire.requestFrom(address, count);  // Read bytes from slave register address
            while (Wire.available()) {
                dest[i++] = Wire.read();
            }         // Put read results in the Rx buffer
        }

}; // class MPU6050
