#include <BMI088.h>

Bmi088Accel accel(Wire, 0x19);

Bmi088Gyro gyro(Wire, 0x69);

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

    checkStatus(gyro.begin(), "gyro");
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
