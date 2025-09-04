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

    pinMode(GYRO_INTERRUPT_PIN, INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN, gyro_drdy, RISING);
}

void loop() 
{
    if (gyro_flag) {

        gyro_flag = false;

        accel.readSensor();

        gyro.readSensor();

        printf("%+05d\n", accel.getAccelX_raw());

        /*
        printf("ax=%+3.3f ay=%+3.3f az=%+3.3f m/s^2 | gx=%+3.3f gy=%+3.3f gz=%+3.3f rad/sec\n", 
                accel.getAccelX_mss(),
                accel.getAccelY_mss(),
                accel.getAccelZ_mss(),
                gyro.getGyroX_rads(),
                gyro.getGyroY_rads(),
                gyro.getGyroZ_rads());*/
    }
}
