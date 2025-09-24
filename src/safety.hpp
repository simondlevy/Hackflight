/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <STM32FreeRTOS.h>

#include <clock.hpp>
#include <datatypes.h>
#include <motors.hpp>

class Safety {

    public:

        Safety(Motors * motors)
        {
            _motors = motors;
        }

        bool isFlying() 
        {
            return _is_flying;
        }

        bool isArmed() 
        {
            return _is_armed;
        }

        void requestArming(const bool doArm) 
        {
            _is_armed = doArm ? _is_safe_to_arm : false;
        }

        void update(
                const uint32_t coreStep,
                const uint32_t timestamp,
                const vehicleState_t & vehicleState)
        { 
            if (Clock::rateDoExecute(CLOCK_RATE, coreStep)) {

                _is_flying = isFlyingCheck(xTaskGetTickCount());

                _is_safe_to_arm =safeAngle(vehicleState.phi) &&
                    safeAngle(vehicleState.theta);
            }
        }

    private:

        static const Clock::rate_t CLOCK_RATE = Clock::RATE_25_HZ;

        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;

        static constexpr float MAX_SAFE_ANGLE = 30;

        Motors * _motors;

        bool _is_flying;

        bool _is_safe_to_arm;

        bool _is_armed;

        static bool safeAngle(float angle)
        {
            return fabs(angle) < MAX_SAFE_ANGLE;
        }

        //
        // We say we are flying if one or more motors are running over the idle
        // thrust.
        //
        bool isFlyingCheck(const uint32_t tick) 
        {
            auto isThrustOverIdle = false;

            for (int i = 0; i < 4; ++i) {
                if (_motors->getRatio(i) > 0) {
                    isThrustOverIdle = true;
                    break;
                }
            }

            static uint32_t latestThrustTick;

            if (isThrustOverIdle) {
                latestThrustTick = tick;
            }

            bool result = false;
            if (0 != latestThrustTick) {
                if ((tick - latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
                    result = true;
                }
            }

            return result;
        }
};

