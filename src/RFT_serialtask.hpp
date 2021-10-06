/*
   Telemetry task for serial comms

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_timertask.hpp"
#include "RFT_parser.hpp"

#include "hf_boards/realboard.hpp"
#include "HF_mixer.hpp"

namespace rft {

    class SerialTask : public TimerTask, public Parser {

        friend class RFTFull;

        protected:

            static constexpr float FREQ = 66;

            bool _useTelemetryPort = false;

            SerialTask(bool secondaryPort=false)
                : TimerTask(FREQ)
            {
                _useTelemetryPort = secondaryPort;
            }

            void update(hf::Board * board, hf::Mixer * mixer, State * state)
            {
                if (!TimerTask::ready(board)) {
                    return;
                }

                hf::RealBoard * realboard = (hf::RealBoard *)board;

                while (realboard->serialAvailable(_useTelemetryPort) > 0) {
                    Parser::parse(realboard->serialRead(_useTelemetryPort));
                }

                while (Parser::availableBytes() > 0) {
                    realboard->serialWrite(Parser::readByte(),
                                           _useTelemetryPort);
                }

                // Support motor testing from GCS
                if (!state->armed) {
                    mixer->runDisarmed();
                }
            }

    };  // SerialTask

} // namespace rft
