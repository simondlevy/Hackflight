/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_pure.hpp>
#include <RFT_board.hpp>
#include <RFT_actuator.hpp>

#include "HF_state.hpp"
#include "HF_receiver.hpp"

namespace hf {

    class Hackflight : public rft::RFTPure {

        public:

            void begin(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
            {
                rft::RFTPure::begin(board, receiver, actuator);
            }

            static void begin(
                    rft::Board * board,
                    Receiver * receiver,
                    rft::Actuator * actuator,
                    rft::Sensor ** sensors,
                    uint8_t sensor_count)
            {
                rft::RFTPure::begin(board, receiver, actuator, sensors, sensor_count);
            }

            void update(rft::Board * board, Receiver * receiver, rft::Actuator * actuator, State * state)
            {
                rft::RFTPure::update(board, receiver, actuator, state);
            }

            void update(
                    rft::Board * board,
                    Receiver * receiver,
                    rft::Actuator * actuator,
                    State * state,
                    rft::Sensor ** sensors,
                    uint8_t sensor_count)
            {
                rft::RFTPure::update(board, receiver, actuator, state, sensors, sensor_count);
            }

    }; // class HackflightPure

} // namespace hf
