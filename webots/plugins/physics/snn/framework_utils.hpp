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
                risp::Processor & proc)
        {
            net.from_json(network_json);

            json jparams = net.get_data("proc_params");

            proc.init(jparams);

            if (!proc.load_network(&net)) {
                throw SRE("loadnetwork() failed");
            }

            track_all_neuron_events(&proc, &net);

            risp::Params params = {};

            params.min_potential = jparams["min_potential"];

#if 0

            discrete = params["discrete"];

            if (params.contains("weights") && params["weights"].size() > 0) {

                weights = params["weights"].get< std::vector<double> >(); 
                inputs_from_weights = params["inputs_from_weights"];
            }

            if (params.contains("threshold_inclusive")) {
                threshold_inclusive = params["threshold_inclusive"];
            }

            if (params.contains("spike_value_factor")) {
                spike_value_factor = params["spike_value_factor"];


            } else if (weights.size() > 0) {
                spike_value_factor = -99999999.99;

            } else {
                spike_value_factor = max_weight;
            } 

            if (params.contains("run_time_inclusive")) {
                run_time_inclusive = params["run_time_inclusive"];
            }

            if (params.contains("fire_like_ravens")) {
                fire_like_ravens = params["fire_like_ravens"];
            }

            if (params.contains("noisy_seed")) {
                noisy_seed = params["noisy_seed"];
            }

            if (params.contains("leak_mode")) {
                const auto mode_string = params["leak_mode"];
                if (mode_string == "all") {
                    leak_mode = LEAK_ALL;
                }
                if (mode_string == "configurable") {
                    leak_mode = LEAK_CONFIG;
                }
            }

            if (params.contains("stds")) {
                stds = params["stds"].get< std::vector<double> >(); 
            }

            if (params.contains("noisy_stddev")) {
                noisy_stddev = params["noisy_stddev"]; 
            }
#endif
        }

    public:

        void foo()
        {
        }

        static void load(
                const char * network_filename,
                Network & net,
                risp::Processor & risp)
        {
            json network_json = {};

            if (!read_json(network_filename, network_json)) {

                printf("usage: ML network_json. Bad json\n");

            } else {

                try {

                    load_network(network_json, net, risp);

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
