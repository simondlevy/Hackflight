#include <BMI088.h>

// internal I^2C
static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t ACCEL_INTERRUPT_PIN = PC13;
static const uint8_t GYRO_INTERRUPT_PIN = PC14;

static const uint8_t ACCEL_ADDRESS = 0x18;
static const uint8_t GYRO_ADDRESS = 0x69;

static TwoWire wire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDRESS);

static Bmi088Gyro gyro(wire, GYRO_ADDRESS);

//static volatile bool accel_flag;
static volatile bool gyro_flag;

/*
static void accel_drdy()
{
    accel_flag = true;
}*/

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

    //accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
    //accel.mapDrdyInt1(true);
    //add_interrupt(ACCEL_INTERRUPT_PIN, accel_drdy);

    check(gyro.begin(), "Gyro Initialization Error");

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    add_interrupt(GYRO_INTERRUPT_PIN, gyro_drdy);
}

void loop() 
{
    if (/*accel_flag &&*/ gyro_flag) {

        //accel_flag = false;
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
