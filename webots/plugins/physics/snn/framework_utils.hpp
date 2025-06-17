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

#include <math.h>

#include <framework.hpp>
#include <risp.hpp>
#include <utils/json_helpers.hpp>

using namespace std;
using namespace neuro;
using nlohmann::json;

class FrameworkUtils {

    private:

        typedef runtime_error SRE;

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
                const json &network_json,
                Network & net,
                Processor **pp,
                risp::Processor & risp)
        {
            (void)risp;

            net.from_json(network_json);

            json proc_params = net.get_data("proc_params");

            const string proc_name = net.get_data("other")["proc_name"];

            Processor * p = Processor::make(proc_name, proc_params);

            if (!p->load_network(&net)) {
                throw SRE("loadnetwork() failed");
            }

            track_all_neuron_events(p, &net);

            *pp = p;
        }

    public:

        static void load(
                const char * network_filename,
                Network & net,
                Processor ** proc,
                risp::Processor & risp)
        {
            json network_json = {};

            if (!read_json(network_filename, network_json)) {

                printf("usage: ML network_json. Bad json\n");

            } else {

                try {

                    load_network(network_json, net, proc, risp);

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
                vector<Spike> & spikes_array,
                const double spike_val=1,
                const bool normalize=true) 
        {
            try {

                p->apply_spike(Spike(net.get_node(spike_id)->input_id,
                            spike_time, spike_val), normalize);

                spikes_array.push_back(Spike(spike_id, spike_time, spike_val));

            } catch (const SRE &e) {
                printf(">>>>>>>> %s\n", e.what());
                exit(0);
            }   
        }

        static string make_viz_message(
                const Network & net,
                const vector<int> counts)
        {
            const size_t n = counts.size();

            string msg = "{\"Event Counts\":[";

            for (size_t i=0; i<n; ++i) {
                char tmp[100] = {};
                const int count = counts[i];
                sprintf(tmp, "%d%s", count, i==n-1 ? "]"  : ", ");
                msg += tmp;
            }

            msg += ", \"Neuron Alias\":[";

            for (size_t i=0; i<n; ++i) {
                char tmp[100] = {};
                sprintf(tmp, "%d%s", 
                        net.sorted_node_vector[i]->id, i==n-1 ? "]" : ", ");
                msg += tmp;
            }

            return msg + "}\n"; 
        }
};
