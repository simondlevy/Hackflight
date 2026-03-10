/*
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

#include <MPU6050.h>

#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/estimators/madgwick/madgwick.hpp>

static MPU6050 _mpu6050;

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

    static auto getVehicleState(const float dt) -> hf::VehicleState
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

        const auto sixaxis = hf::SixAxis(gyro, accel);

        static hf::MadgwickFilter  _madgwick;

        _madgwick = hf::MadgwickFilter::run(
                _madgwick, dt, 
                {sixaxis.gyro.x, -sixaxis.gyro.y, -sixaxis.gyro.z},
                {-sixaxis.accel.x, sixaxis.accel.y, sixaxis.accel.z});

        const float dx = 0;
        const float dy = 0;
        const float z = 0;
        const float dz = 0;
        const auto phi = _madgwick.angles.x;
        const auto theta = _madgwick.angles.y;
        const auto psi = _madgwick.angles.z;
        const auto dphi = sixaxis.gyro.x;
        const auto dtheta = sixaxis.gyro.y;
        const auto dpsi = -sixaxis.gyro.z;

        return hf::VehicleState(
                dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi);
    }
}
