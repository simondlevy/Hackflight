#include <SPI.h>

#include <BMI088.h>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;
static const uint8_t GYRO_INT_PIN = PC14;

static SPIClass spi;

Bmi088 bmi(spi, ACCEL_CS_PIN, GYRO_CS_PIN);

void drdy()
{
    bmi.readSensor();

    Serial.print(bmi.getAccelX_mss());
    Serial.print("\t");
    Serial.print(bmi.getAccelY_mss());
    Serial.print("\t");
    Serial.print(bmi.getAccelZ_mss());
    Serial.print("\t");
    Serial.print(bmi.getGyroX_rads());
    Serial.print("\t");
    Serial.print(bmi.getGyroY_rads());
    Serial.print("\t");
    Serial.print(bmi.getGyroZ_rads());
    Serial.print("\t");
    Serial.print(bmi.getTemperature_C());
    Serial.print("\n");
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
    while(!Serial) {}

    spi.setSCLK(PB13);
    spi.setMISO(PB14);
    spi.setMOSI(PB15);

    check(bmi.begin(),"IMU Initialization Error");

    check(bmi.setRange(Bmi088::ACCEL_RANGE_6G,Bmi088::GYRO_RANGE_500DPS),
            "Failed to set ranges");

    check(bmi.setOdr(Bmi088::ODR_400HZ), "Failed to set ODR");

    check(bmi.mapSync(Bmi088::PIN_3), "Failed to map sync pin");

    check(bmi.mapDrdy(Bmi088::PIN_2), "Failed to map data ready pin");

    check(bmi.pinModeDrdy(Bmi088::PUSH_PULL,Bmi088::ACTIVE_HIGH),
            "Failed to setup data ready pin");

    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, drdy, RISING);
}

void loop() 
{
}
