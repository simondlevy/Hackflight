#include <SPI.h>

#include <BMI088.h>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;
static const uint8_t GYRO_INT_PIN = PC14;

static SPIClass spi;

Bmi088Accel accel(spi, ACCEL_CS_PIN);

Bmi088Gyro gyro(spi, GYRO_CS_PIN);

static volatile uint32_t gyro_interrupt_count;

static void gyro_drdy()
{
    gyro_interrupt_count++;
}

static void init(const int status, const char * what)
{
    if (status < 0) {
        Serial.print("Error initializing ");
        Serial.println(what);
        while (true) ;
    }
}

void setup() 
{
    Serial.begin(115200);

    while (!Serial) ;

    spi.setSCLK(PB13);
    spi.setMISO(PB14);
    spi.setMOSI(PB15);

    init(accel.begin(), "accel");

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);

    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, gyro_drdy, RISING);

    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    init(gyro.begin(), "gyro");
}

void loop() 
{
    accel.readSensor();

    accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);

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
