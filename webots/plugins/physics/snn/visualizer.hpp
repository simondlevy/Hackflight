/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <vector>

#include <posix-utils/server.hpp>

class Visualizer {

    private:

        static const uint16_t VIZ_PORT = 8100;

        static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

        const char * NETWORK_PATH =
            "%s/Desktop/2025-diff-network/levy/max_%d.txt";

        float _max_spike_time;

        ServerSocket _spikeServer;

    public:

        void init(const float max_spike_time)
        {
            _max_spike_time = max_spike_time;

            _spikeServer.open(VIZ_PORT);
            _spikeServer.acceptClient();
        }

        void send_spikes(
                const int i1_spike_time,
                const int i2_spike_time,
                const int d1_spike_time,
                const int d2_spike_time,
                const int s_spike_time,
                const int o_spike_time)
        {
            const float i_scale = 50 / _max_spike_time;
            const float d_scale = 250 / _max_spike_time;
            const float o_scale = 25 / _max_spike_time;
            const float s_bias = 0.8 * _max_spike_time;
            const float s_scale = 125 / _max_spike_time;

            static uint32_t _tick;

            if (_tick++ == VIZ_SEND_PERIOD) {

                const std::vector<int> counts = {
                    (int)(i1_spike_time * i_scale),
                    (int)(i2_spike_time * i_scale),
                    1,
                    (int)(d1_spike_time * d_scale),
                    (int)(d2_spike_time * d_scale),
                    (int)((o_spike_time - _max_spike_time) * o_scale),
                    (int)((s_spike_time - s_bias) * s_scale)
                };

                //const string msg = _framework.make_viz_message(counts);

                //_spikeServer.sendData((uint8_t *)msg.c_str(), msg.length());

                _tick = 0;
            }
        }
};
