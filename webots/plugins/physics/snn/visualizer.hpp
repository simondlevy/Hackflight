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

#include <posix-utils/server.hpp>

#if 0
class Visualizer {

    private:

        static const uint16_t VIZ_PORT = 8100;

        static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

        const char * NETWORK_PATH =
            "%s/Desktop/2025-diff-network/levy/max_%d.txt";

        // For visualization
        static constexpr double I_SCALE = 50 / MAX_SPIKE_TIME;
        static constexpr double D_SCALE = 250 / MAX_SPIKE_TIME;
        static constexpr double O_SCALE = 25 / MAX_SPIKE_TIME;
        static constexpr double S_BIAS = 0.8 * MAX_SPIKE_TIME;
        static constexpr double S_SCALE = 125 / MAX_SPIKE_TIME;

        Framework _framework = Framework(MAX_SPIKE_TIME);

        ServerSocket _spikeServer;

        void send_spikes_to_visualizer(
        {
            static uint32_t _count;

            if (_count++ == VIZ_SEND_PERIOD) {
                const vector<int> tmp = _framework.get_neuron_counts();
                const vector<int> counts = {
                    (int)(spike_time_1 * I_SCALE),
                    (int)(spike_time_2 * I_SCALE),
                    1,
                    (int)(tmp[3] * D_SCAE),
                    (int)(tmp[4] * D_SCALE),
                    (int)((out - MAX_SPIKE_TIME) * O_SCALE),
                    (int)((tmp[6] - S_BIAS) * S_SCALE)
                };

                const string msg = _framework.make_viz_message(counts);
                _spikeServer.sendData((uint8_t *)msg.c_str(), msg.length());
                _count = 0;
            }
        }

    public:

        void init(const bool visualize=false)
        {
            // Load the network
            char path[256] = {};
            sprintf(path, NETWORK_PATH, getenv("HOME"), (int)MAX_SPIKE_TIME);
            _framework.load(path);

            // Listen for and accept connections from vizualization client
            if (visualize) {
                _spikeServer.open(VIZ_PORT);
                _spikeServer.acceptClient();
            }

            _visualize = visualize;
        }

        double run(const float inp1, const float inp2)
        {
            // Turn the inputs into spikes
            const double spike_time_1 = _framework.value_to_spike_time(inp1);
            const double spike_time_2 = _framework.value_to_spike_time(inp2);

            // Apply the spikes to the network
            _framework.apply_spike(0, spike_time_1);
            _framework.apply_spike(1, spike_time_2);
            _framework.apply_spike(2, 0);

            // Run the network
            _framework.run(3 * MAX_SPIKE_TIME + 2);

            // Get the output network's firing time
            const double out = _framework.get_output_vector()[0];
            const double time = out == MAX_SPIKE_TIME + 1 ? -2 : out;

            // Send spikes to the visualizer if indicated
            if (_visualize) {
                send_spikes_to_visualizer(spike_time_1, spike_time_2, out);
            }

            // Convert the firing time to a difference in [-2,+2]
            return (time-MAX_SPIKE_TIME)*2 / MAX_SPIKE_TIME - 2;
        }
};
#endif
