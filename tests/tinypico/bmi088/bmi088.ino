#include <BMI088.h>

static const uint8_t ACCEL_ADDR = 0x19;
static const uint8_t GYRO_ADDR = 0x69;
static const uint8_t GYRO_INT_PIN = 4;

Bmi088Accel accel(Wire, ACCEL_ADDR);

Bmi088Gyro gyro(Wire, GYRO_ADDR);

static volatile bool got_gyro_interrupt;

static void handle_gyro_interrupt()
{
    got_gyro_interrupt = true;
}

static void checkStatus(const int status, const char * sensorname)
{
    while (status < 0) {

        Serial.print(sensorname);
        Serial.print(" initialization failed with status ");
        Serial.println(status);
        delay(500);
    }
}

void setup() 
{
    int status;

    Serial.begin(115200);

    Wire.begin();

    delay(100);

    checkStatus(accel.begin(), "accel");

    accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);

    checkStatus(gyro.begin(), "gyro");

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    gyro.pinModeInt3(Bmi088Gyro::PIN_MODE_PUSH_PULL,
            Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, handle_gyro_interrupt, RISING);
}

void loop() 
{
    if (got_gyro_interrupt) {

        got_gyro_interrupt = false;

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
    }
}
