#include <BMI088.h>

static const uint8_t GYRO_INTERRUPT_PIN = 6;

static Bmi088Accel accel(Wire,0x19);

static Bmi088Gyro gyro(Wire,0x69);

static volatile bool gyro_flag;

void gyro_drdy()
{
    gyro_flag = true;
}

static void reportForever(const char * msg)
{
  while (true) {
    Serial.println(msg);
  }
}

void setup() 
{

    Serial.begin(0);

    if (accel.begin() < 0) {
        reportForever("Accel Initialization Error");
    }

    if (gyro.begin() < 0) {
        reportForever("Gyro Initialization Error");
    }

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    pinMode(GYRO_INTERRUPT_PIN, INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN, gyro_drdy, RISING);  
}

void loop() 
{
    //if (gyro_flag) {

        gyro_flag = false;

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
    //}
}
