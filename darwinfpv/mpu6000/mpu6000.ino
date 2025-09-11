#include <mpu6x00.h>

static const uint8_t SCLK_PIN = PA5;
static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;

static const uint8_t CS_PIN  = PA4;
static const uint8_t INT_PIN = PA1;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static Mpu6500 imu = Mpu6500(spi, CS_PIN);

static volatile bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static void errorForever(void)
{
    while (true) {
        Serial.println("Error initializing IMU");
        delay(500);
    }
}

void setup(void)
{
    Serial.begin(115200);

    spi.begin();

    if (!imu.begin()) {
        errorForever();
    }

    pinMode(INT_PIN, INPUT);
    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void)
{
    if (gotInterrupt) {

        imu.readSensor();

        Serial.print(imu.getRawGyroX());
        Serial.print("  ");
        Serial.print(imu.getRawGyroY());
        Serial.print("  ");
        Serial.print(imu.getRawGyroZ());
        Serial.println(); 

        gotInterrupt = false;
    }
}

