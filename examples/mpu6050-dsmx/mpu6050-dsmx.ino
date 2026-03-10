/*
   Hackflight for Teensy 4.0 

   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <dshot-teensy4.hpp>  
#include <dsmrx.hpp>  
#include <MPU6050.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/estimators/madgwick/madgwick.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/dsmx.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

static MPU6050 _mpu6050;


// Motors ---------------------------------------------------------

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

// IMU ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384;

static constexpr float B_ACCEL = 0.14;     
static constexpr float B_GYRO = 0.1;       

static constexpr float ACCEL_ERROR_X = 0.0;
static constexpr float ACCEL_ERROR_Y = 0.0;
static constexpr float ACCEL_ERROR_Z = 0.0;
static constexpr float GYRO_ERROR_X = 0.0;
static constexpr float GYRO_ERROR_Y= 0.0;
static constexpr float GYRO_ERROR_Z = 0.0;

// LED -------------------------------------------------------------

static hf::LED _led = hf::LED(13);

// PID control -----------------------------------------------------

static hf::StabilizerPid _stabilizerPid;

// Motor mixing ----------------------------------------------------

static hf::Mixer _mixer;

// FAFO -----------------------------------------------------------

static const uint32_t LOOP_FREQ_HZ = 2000;

// Helper functions ------------------------------------------------

static void initImu()
{

    Wire.begin();
    Wire.setClock(1000000); 

    _mpu6050.initialize();

    if (_mpu6050.testConnection() == false) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
    }

    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

namespace hf {

    class ThreeAxisFilter {

        public:

            ThreeAxisFilter() : _prev(Vec3(0, 0, 0)) {}

            ThreeAxisFilter& operator=(const ThreeAxisFilter& other) = default;

            Vec3 run(
                    const Vec3 & raw,
                    const Vec3 & error,
                    const float scale,
                    const float coeff)
            {
                const auto curr = raw / scale - error;

                const auto output = _prev * (1 - coeff) + curr * coeff;

                _prev = curr;

                return output;
            }

        private:

            Vec3 _prev;
    };

}

static auto readImu() -> hf::SixAxis
{
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    static hf::ThreeAxisFilter _gyroFilter;

    const auto gyro = _gyroFilter.run(
            hf::Vec3(gx, gy, gz),
            hf::Vec3(GYRO_ERROR_X, GYRO_ERROR_Y, GYRO_ERROR_Z),
            GYRO_SCALE_FACTOR,  B_GYRO);

    static hf::ThreeAxisFilter _accelFilter;

    const auto accel = _accelFilter.run(
            hf::Vec3(ax, ay, az),
            hf::Vec3(ACCEL_ERROR_X, ACCEL_ERROR_Y, ACCEL_ERROR_Z),
            ACCEL_SCALE_FACTOR,  B_ACCEL);

    return hf::SixAxis(gyro, accel);

    /*
    static float accel_x_prev, accel_y_prev, accel_z_prev;
    static float gyro_x_prev, gyro_y_prev, gyro_z_prev;

    (void)gyro;
    (void)accel;

    auto accel_x = ax / ACCEL_SCALE_FACTOR - ACCEL_ERROR_X; 
    auto accel_y = ay / ACCEL_SCALE_FACTOR - ACCEL_ERROR_Y;
    auto accel_z = az / ACCEL_SCALE_FACTOR - ACCEL_ERROR_Z;

    accel_x = (1.0 - B_ACCEL)*accel_x_prev + B_ACCEL*accel_x;
    accel_y = (1.0 - B_ACCEL)*accel_y_prev + B_ACCEL*accel_y;
    accel_z = (1.0 - B_ACCEL)*accel_z_prev + B_ACCEL*accel_z;

    accel_x_prev = accel_x;
    accel_y_prev = accel_y;
    accel_z_prev = accel_z;

    auto gyro_x = gx / GYRO_SCALE_FACTOR - GYRO_ERROR_X; 
    auto gyro_y = gy / GYRO_SCALE_FACTOR - GYRO_ERROR_Y;
    auto gyro_z = gz / GYRO_SCALE_FACTOR - GYRO_ERROR_Z;

    gyro_x = (1.0 - B_GYRO)*gyro_x_prev + B_GYRO*gyro_x;
    gyro_y = (1.0 - B_GYRO)*gyro_y_prev + B_GYRO*gyro_y;
    gyro_z = (1.0 - B_GYRO)*gyro_z_prev + B_GYRO*gyro_z;

    gyro_x_prev = gyro_x;
    gyro_y_prev = gyro_y;
    gyro_z_prev = gyro_z;

    return hf::SixAxis({gyro_x, gyro_y, gyro_z}, {accel_x, accel_y, accel_z});*/
}

static void runDelayLoop(const uint32_t usec_curr)
{
    float invFreq = 1.0 / LOOP_FREQ_HZ * 1000000.0;
    uint32_t checker = micros();

    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}

static void getVehicleState(const float dt, hf::VehicleState & state)
{
    const auto sixaxis = readImu();

    static hf::MadgwickFilter  _madgwick;

    _madgwick = hf::MadgwickFilter::run(
            _madgwick, dt, 
                {sixaxis.gyro.x, -sixaxis.gyro.y, -sixaxis.gyro.z},
                {-sixaxis.accel.x, sixaxis.accel.y, sixaxis.accel.z});

    state.phi = _madgwick.angles.x;
    state.theta = _madgwick.angles.y;
    state.psi = _madgwick.angles.z;

    state.dphi = sixaxis.gyro.x;
    state.dtheta = sixaxis.gyro.y;
    state.dpsi = -sixaxis.gyro.z;
}

// Main ----------------------------------------------------------------------

void setup()
{
    rx_init();

    Serial1.begin(115000);

    initImu();

    delay(10);

    _motors.arm(); 

    _led.begin(); 
}

void loop()
{
    const auto usec_curr = micros();      

    const auto dt = hf::Timer::getDt();

    _led.blink(); 

    rx_read();

    const auto setpoint = hf::mksetpoint(rx_chanvals);

    hf::VehicleState state = {};
    getVehicleState(dt, state);

    //hf::Debugger::debug(rx_is_armed, setpoint, state);
    hf::Debugger::profile();

    _stabilizerPid = hf::StabilizerPid::run(_stabilizerPid,
            !rx_is_throttle_down, dt, state, setpoint);

    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(rx_is_armed, _mixer.motorvals);

    runDelayLoop(usec_curr); 
}
