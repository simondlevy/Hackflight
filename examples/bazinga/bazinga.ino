#include <SPI.h>

#include <BMI088.h>

static constexpr Bmi088Gyro::Range kGyroRange = Bmi088Gyro::RANGE_2000DPS;
static constexpr Bmi088Accel::Range kAccelRange = Bmi088Accel::RANGE_24G;

// The SDO pin should either be pulled low for the 0x18/0x68
// addresses, high for 0x19/0x69
static Bmi088Accel accel_ = Bmi088Accel(Wire, 0x18);
static Bmi088Gyro gyro_ = Bmi088Gyro(Wire, 0x68);

static auto ImuOkay(const int status) -> bool
{
    return status >= 0;
}

void setup()
{
    if (!(ImuOkay(gyro_.begin()) &&

        ImuOkay(accel_.begin()) &&

        ImuOkay(gyro_.setOdr(Bmi088Gyro::ODR_2000HZ_BW_532HZ)) &&

        ImuOkay(gyro_.setRange(kGyroRange)) &&

        ImuOkay(gyro_.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

        ImuOkay(gyro_.mapDrdyInt3(true)) &&

        ImuOkay(accel_.setOdr(Bmi088Accel::ODR_800HZ_BW_80HZ)) &&

        ImuOkay(accel_.setRange(kAccelRange)))) {

        while (true) {
            Serial.printf("Unable to start Imu\n");
            delay(500);
        }
    }
}

void loop()
{
    static uint32_t count_;
    static uint32_t msec_;

    if (gyro_.getDrdyStatus()) {
        count_++;
    }

    const auto msec = millis();

    if (msec - msec_ > 1000) {
        printf("%lu\n", count_);
        count_ = 0;
        msec_ = msec;
    };
}
