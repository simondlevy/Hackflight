#include <SPI.h>

#include <BMI088.h>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;
static const uint8_t GYRO_INT_PIN = PC14;

static const uint8_t MISO_PIN = PB14;
static const uint8_t MOSI_PIN = PB15;
static const uint8_t SCLK_PIN = PB13;

static SPIClass spi;

Bmi088Accel accel(spi, ACCEL_CS_PIN);

Bmi088Gyro gyro(spi, GYRO_CS_PIN);

static volatile uint32_t gyro_interrupt_count;

static void gyro_drdy()
{
    gyro_interrupt_count++;
}

static void check(const int status, const char * msg)
{
    if (status < 0) {
        Serial.println(msg);
        while (true) ;
    }
}


void setup() 
{
    Serial.begin(115200);

    spi.setMISO(MISO_PIN);
    spi.setMOSI(MOSI_PIN);
    spi.setSCLK(SCLK_PIN);

    check(gyro.begin(), "Gyro initialization error");

    check(accel.begin(), "Accel initialization error");

    check(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ), "Failed to set gyro ODR");

    check(gyro.setRange(Bmi088Gyro::RANGE_2000DPS), "Failed to set gyro range");

    check(gyro.pinModeInt3(Bmi088Gyro::PIN_MODE_PUSH_PULL,
                Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH),
            "Failed to setup data-ready pin");

    check(gyro.mapDrdyInt3(true), "Failed to map data-ready pin");

    check(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ),
            "Failed to set accel ODR");

    check(accel.setRange(Bmi088Accel::RANGE_24G), "Failed to set accel range");

    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, gyro_drdy, RISING);
}

void loop() 
{
    accel.readSensor();

    gyro.readSensor();

    Serial.print(accel.getAccelX_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelY_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelZ_mss());
    Serial.print("\t");
    Serial.print(gyro.getGyroX_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroY_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroZ_rads());
    Serial.print("\t");
    Serial.print(accel.getTemperature_C());
    Serial.print("\t");
    Serial.println(gyro_interrupt_count);

    delay(20);
}
