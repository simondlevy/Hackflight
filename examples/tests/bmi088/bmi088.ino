#include <Wire.h>

#include <BMI088.h>

static const uint8_t GYRO_INTERRUPT_PIN = 6;

static const uint8_t ACCEL_ADDRESS = 0x19;
static const uint8_t GYRO_ADDRESS = 0x69;

static Bmi088Accel accel(Wire, ACCEL_ADDRESS);

static Bmi088Gyro gyro(Wire, GYRO_ADDRESS);

static volatile bool gyro_flag;

static void gyro_drdy()
{
    gyro_flag = true;
}

static void check(const int status, const char * msg)
{
    if (status < 0) {
        Serial.println(msg);
        while (true) ;
    }
}

static void add_interrupt(const uint8_t pin, void (*handler)())
{
    pinMode(pin, INPUT);
    attachInterrupt(pin, handler, RISING);
}

void setup() 
{
    Serial.begin(115200);

    while (!Serial) ;

    check(accel.begin(), "Accel Initialization Error");

    accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);

    check(gyro.begin(), "Gyro Initialization Error");

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    add_interrupt(GYRO_INTERRUPT_PIN, gyro_drdy);
}

void loop() 
{
    if (gyro_flag) {

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
    }
}
