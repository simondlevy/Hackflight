/**
 * Copyright 2025 Simon D. Levy
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

#include <iostream>
#include <vector>
#include <string>

#include <posix-utils/server.hpp>

#include <control/snn/helper.hpp>

class ClosedLoopControl {

    private:

        static const int VIZ_PORT = 8100;

        static const int VIZ_SEND_PERIOD = 50; // ticks

        SnnHelper _helper;

        ServerSocket _spikeServer;

        static std::string make_viz_message(const std::vector<int> counts)
        {
            const int n = counts.size();

            std::string msg = "{\"Event Counts\":[";
            for (int i=0; i<n; ++i) {
                char tmp[100] = {};
                const int count = counts[i];
                sprintf(tmp, "%d%s", count, i==n-1 ? "]"  : ", ");
                msg += tmp;
            }

            /*
            std::string msg = "{\"Event Times\":[";
            for (int i=0; i<n; ++i) {
                char tmp[100] = {};
                const int time = 100;
                sprintf(tmp, "[%d]%s", time, i==n-1 ? "]"  : ", ");
                msg += tmp;
            }*/

            msg += ", \"Neuron Alias\":[";
            for (int i=0; i<n; ++i) {
                char tmp[100] = {};
                sprintf(tmp, "%d%s", i, i==n-1 ? "]" : ", ");
                msg += tmp;
            }

            return msg + "}\n";
        }

    public:

        void init()
        {
            _spikeServer.open(VIZ_PORT);
            _spikeServer.acceptClient();
        }

        void run(
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                const float landingAltitudeMeters,
                demands_t & demands)
        {
            _helper.run(dt, hovering, vehicleState, openLoopDemands,
                    landingAltitudeMeters, demands); 

            static int _tick;

            if (_tick++ == VIZ_SEND_PERIOD) {

                const std::vector<int> counts = {
                    _helper.get_i1_spike_count(),
                    _helper.get_i2_spike_count(),
                    _helper.get_s_spike_count(),
                    _helper.get_d1_spike_count(),
                    _helper.get_d2_spike_count(),
                    _helper.get_o_spike_count(),
                    _helper.get_s2_spike_count()
                };

                const std::string msg = make_viz_message(counts);

                _spikeServer.sendData((uint8_t *)msg.c_str(), msg.length());

                _tick = 0;
            }
        }

};

static ClosedLoopControl _closedLoopControl;
