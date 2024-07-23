#pragma once

#include <math.h>
#include <datatypes.h>

#ifndef M_PI
#define M_PI 3.1415928f
#endif

class Utils {

    public:

        static constexpr float DEG2RAD = M_PI / 180.0f;
        static constexpr float RAD2DEG = 180.0f / M_PI;
        static constexpr float GS2MSS = 9.81;

        static void quat2euler(const quat_t & q, float & phi, float & theta, float & psi)
        {
            // We swap the X and Y axes and negate Y for nose-down positive.
            phi = RAD2DEG * asin((-2) * (q.x*q.z - q.w*q.y));

            theta = -RAD2DEG * atan2((2 * (q.y*q.z + q.w*q.x)),
                    (q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z));

            // Negate for nose-right positive
            psi = -RAD2DEG * atan2((2 * (q.x*q.y + q.w*q.z)),
                    (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z));
        }

        static const float fmax(const float val, const float maxval)
        {
            return val > maxval ? maxval : val;
        }

        static const float fmin(const float val, const float maxval)
        {
            return val < maxval ? maxval : val;
        }

        static float rescale(
                const float value,
                const float oldmin,
                const float oldmax,
                const float newmin,
                const float newmax)
        {
            return (value - oldmin) / (oldmax - oldmin) * 
                (newmax - newmin) + newmin;
        }

        static float square(const float x)
        {
            return x * x;
        }

        static float fconstrain(
                const float val, const float minval, const float maxval)
        {
            return val < minval ? minval : val > maxval ? maxval : val;
        }

        static uint8_t u8constrain(
                const uint8_t val, const uint8_t minval, const uint8_t maxval)
        {
            return val < minval ? minval : val > maxval ? maxval : val;
        }

        static bool in_deadband(const float val, const float band)
        {
            return fabs(val) < band;
        }
};


