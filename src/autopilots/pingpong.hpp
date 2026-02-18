/* 
 *  "Ping-Pong" autopilot using 1x1 rangefinder
 *
 *  Copyright (C) 2026 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// C
#include <stdlib.h>

// Hackflight
#include <datatypes.h>
#include <simulator/dynamics.hpp>

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

namespace hf {

    class PingPongAutopilot {

        private:

            static constexpr int WALL_PROXIMITY_MM = 200;
            static constexpr float SPEED = 0.5;

        public:

            int distance_forward_mm;
            int distance_backward_mm;

            void init()
            {
                srand(time(NULL)); 
            }

            void getSetpoint(const float dydt, hf::demands_t & setpoint) 
            {
                const int8_t direction = 

                    // Motionless on startup; move in a random direction
                    // (forward or backward)
                    fabs(dydt) < 1e-6 ? 2 * (rand() % 2) - 1 :

                    // Close to forward wall, go backward
                    distance_forward_mm < WALL_PROXIMITY_MM ? -1 :

                    // Close to backward wall, go forward
                    distance_backward_mm < WALL_PROXIMITY_MM ? +1 :

                    // Otherwise, continue in same direction
                    dydt > 0 ? -1 : +1;

                setpoint.pitch = direction * SPEED;
            }

            void readSensors(
                    simsens::Robot & robot,
                    simsens::World & world,
                    const simsens::pose_t & pose)
            {
                distance_forward_mm = readRangefinder("VL53L1-forward", robot,
                        world, pose);
                distance_backward_mm = readRangefinder("VL53L1-backward", robot,
                        world, pose);
            }

        private:

            static int readRangefinder(
                    const string name,
                    simsens::Robot & robot,
                    simsens::World & world,
                    const simsens::pose_t & pose)
            {
                auto rangefinder = robot.rangefinders[name];

                int distance_mm = 0;

                rangefinder->read(pose, world, &distance_mm);

                return distance_mm;
            }

    };

}
