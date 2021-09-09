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

            _aRes = (1 << ((uint8_t)ascale + 1)) / 32768.0;
            _gRes = (250 * ((uint8_t)ascale + 1)) / 32768.0;
        }

        void begin()
        {
            // wake up device-don't need this here if using calibration function below
            writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
            delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

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

        bool dataReady(void)
        {
            return (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01);
        }

        void readData(float & ax, float &ay, float &az, float &gx, float &gy, float &gz)
        {
            readData(ax, ay, az, ACCEL_XOUT_H, _aRes);
            readData(gx, gy, gz, GYRO_XOUT_H, _gRes);
        }

    private:

        static const uint8_t  MPU6050_ADDRESS = 0x68;  // Device address

        static const uint8_t  SMPLRT_DIV     = 0x19;
        static const uint8_t  CONFIG         = 0x1A;
        static const uint8_t  GYRO_CONFIG    = 0x1B;
        static const uint8_t  ACCEL_CONFIG   = 0x1C;
        static const uint8_t  INT_PIN_CFG    = 0x37;
        static const uint8_t  INT_ENABLE     = 0x38;
        static const uint8_t  DMP_INT_STATUS = 0x39;  // Check DMP interrupt;
        static const uint8_t  INT_STATUS     = 0x3A;
        static const uint8_t  ACCEL_XOUT_H   = 0x3B;
        static const uint8_t  GYRO_XOUT_H    = 0x43;
        static const uint8_t  PWR_MGMT_1     = 0x6B; // Device defaults to the SLEEP mode;

        ascale_t _ascale;
        gscale_t _gscale;

        float _aRes = 0;
        float _gRes = 0;

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
