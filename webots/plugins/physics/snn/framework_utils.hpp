/**
 * TeNNLab Open-Source Framework utilities
 * 
 * Copyright (C) 2025 James Plank, Simon D. Levy
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

#include <fstream>

#include <framework_json.hpp>
#include <risp.hpp>

using namespace std;
using namespace neuro;
using nlohmann::json;

class FrameworkUtils {

    private:

        static bool read_json(const char * filename, json &rv)
        {
            bool success = true;

            string s = {};

            ifstream fin = {};

            rv.clear();

            fin.clear();

            fin.open(filename);

            try {
                fin >> rv; success = true;
            } catch(...) {
                success = false;
            }

            fin.close();

            return success;
        }

        static void load_network(
                const json &j,
                Network & net,
                risp::Processor & proc)
        {
            NetworkLoader::load(j, &net, proc);

        }

    public:

        static void load(
                const char * network_filename,
                Network & net,
                risp::Processor & proc)
        {
            json j = {};

            if (!read_json(network_filename, j)) {

                printf("usage: ML j. Bad json\n");

            } else {

                try {

                    load_network(j, net, proc);

                } catch (const SRE &e) {
                    printf("%s\n",e.what());
                } catch (...) {
                    printf("Unknown error when making processor\n");
                }
            }
        }

        static double get_spike_time(const double inp, const double max)
        {
            return round(max * (1 - inp) / 2);
        }

        static void apply_spike(
                Network & net,
                Processor *p,
                const int spike_id,
                const double spike_time,
                const double spike_val=1,
                const bool normalize=true) 
        {
            try {

                p->apply_spike(Spike(net.get_node(spike_id)->input_id,
                            spike_time, spike_val), normalize);

            } catch (const SRE &e) {
                printf(">>>>>>>> %s\n", e.what());
                exit(0);
            }   
        }

};
