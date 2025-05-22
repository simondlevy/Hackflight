#include <BMI088.h>

static SPIClass myspi = SPIClass (PB15, PB14, PB13); // MOSI, MISO, SCLK
static Bmi088Accel accel(myspi, PB1); // CS
static Bmi088Gyro gyro(myspi, PB0);   // CS

static bool gotGyroInterrupt;

static void gyroDrdy()
{
    gotGyroInterrupt = true;
}

void setup() 
{
    Serial.begin(115200);

    int status = 0;

    status = accel.begin();
    while (status < 0) {
        Serial.println("Accel Initialization Error");
        Serial.println(status);
        delay(500);
    }

    accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
    accel.setRange(Bmi088Accel::RANGE_24G);

    status = gyro.begin();
    while (status < 0) {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
        delay(500);
    }

    gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    gyro.setRange(Bmi088Gyro::RANGE_2000DPS);

    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    pinMode(PC14, INPUT);
    attachInterrupt(PC14, gyroDrdy, RISING);
}

void loop() 
{
    if (gotGyroInterrupt) {

        gotGyroInterrupt = false;

        gyro.readSensor();

        Serial.print(accel.getAccelX_mss());
        Serial.print("\t");
        Serial.print(accel.getAccelY_mss());
        Serial.print("\t");
        Serial.print(accel.getAccelZ_mss());
        Serial.print("\t");

        accel.readSensor();

        Serial.print(gyro.getGyroX_rads());
        Serial.print("\t");
        Serial.print(gyro.getGyroY_rads());
        Serial.print("\t");
        Serial.print(gyro.getGyroZ_rads());
        Serial.print("\t");

        Serial.print(accel.getTemperature_C());

        Serial.print("\n");
    }

}
