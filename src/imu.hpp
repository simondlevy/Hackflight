/*
   IMU class

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

namespace hf {

    class IMU {

        // NB: Euler angles and gyrometer should exhibit values as follows:
        //
        // GX: roll right +, left -
        // GY: pitch forward -, back +
        // GZ: yaw right +, left -
        //
        // EX: roll right +, left -
        // EY: pitch forward +, back -
        // EY: clockwise increase, counter decrease (mod 2PI)

        friend class Hackflight;
        friend class Quaternion;
        friend class Gyrometer;

        protected:

            // Core functionality
            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) = 0;
            virtual bool getGyrometer(float & gx, float & gy, float & gz) = 0;

            // Adjustment for non-standard mounting
            virtual void adjustEulerAngles(float & x, float & y, float & z) { (void)x; (void)y; (void)z; }
            virtual void adjustGyrometer(float & x, float & y, float & z) { (void)x; (void)y; (void)z;  }

            // Required by some IMUs
            virtual void begin(void) { }

            // Support for additional surface-mount sensors
            virtual bool  getAccelerometer(float & ax, float & ay, float & az) { (void)ax; (void)ay; (void)az; return false; }
            virtual bool  getMagnetometer(float & mx, float & my, float & mz) { (void)mx; (void)my; (void)mz; return false; }
            virtual bool  getBarometer(float & pressure) { (void)pressure;  return false; }

            // Helper
            static void swap(float & a, float & b)
            {
                float tmp = a;
                a = b;
                b = tmp;
            }

    }; // class IMU

} // namespace hf
