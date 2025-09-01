#include <SPI.h>

#include <BMI088.h>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;

static SPIClass spi;

Bmi088Accel accel(spi, ACCEL_CS_PIN);

Bmi088Gyro gyro(spi, GYRO_CS_PIN);

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
    int status;

    Serial.begin(115200);

    while (!Serial) ;

    spi.setSCLK(PB13);
    spi.setMISO(PB14);
    spi.setMOSI(PB15);

    init(accel.begin(), "accel");

    init(gyro.begin(), "gyro");
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
    Serial.print("\n");

    delay(20);
}
