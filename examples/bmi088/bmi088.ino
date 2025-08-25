#include <BMI088.h>

// internal I^2C
static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t ACCEL_ADDRESS = 0x18;
static const uint8_t GYRO_ADDRESS = 0x69;

static TwoWire wire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDRESS);

static Bmi088Gyro gyro(wire, GYRO_ADDRESS);

static volatile bool accel_flag, gyro_flag;

static void accel_drdy()
{
    accel_flag = true;
}

static void gyro_drdy()
{
    gyro_flag = true;
}

void setup() 
{
    int status;

    Serial.begin(115200);
    while(!Serial) {}

    status = accel.begin();
    if (status < 0) {
        Serial.println("Accel Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);
    status = accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
    status = accel.mapDrdyInt1(true);


    status = gyro.begin();
    if (status < 0) {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    status = gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    status = gyro.mapDrdyInt3(true);

    pinMode(PC13,INPUT);
    attachInterrupt(PC13,accel_drdy,RISING);

    pinMode(PC14,INPUT);
    attachInterrupt(PC14,gyro_drdy,RISING);  
}

void loop() 
{
    if (accel_flag && gyro_flag) {

        accel_flag = false;
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
