/**
 * TeNNLab Open-Source Framework utilities
 * 
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

#include <math.h>

#include <framework.hpp>
#include <utils/json_helpers.hpp>

using namespace std;
using namespace neuro;
using nlohmann::json;

class Framework {

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

        static double cap(const double val)
        {
            return val > +1 ? +1 : val < -1 ? -1 : val;
        }

        double _max_spike_time;
        Network * _net;
        Processor * _proc;

    public:

        Framework(const double max_spike_time)
            : _max_spike_time(max_spike_time)
        {
        }

        void load(const char * network_filename)
        {
            json network_json = {};

            if (read_json(network_filename, network_json)) {

                try {

                    _net = new Network();

                    _net->from_json(network_json);

                    json proc_params = _net->get_data("proc_params");

                    string proc_name = _net->get_data("other")["proc_name"];

                    _proc = Processor::make(proc_name, proc_params);

                    if (_proc->get_network_properties().as_json() !=
                            _net->get_properties().as_json()) {
                    }

                    if (!_proc->load_network(_net)) {
                        throw SRE("loadnetwork() failed");
                    }
                    track_all_neuron_events(_proc, _net);

                } catch (const SRE &e) {
                    printf("%s\n",e.what());
                } catch (...) {
                    printf("Unknown error when making processor\n");
                }
            } 
            
            else {

                printf("usage: ML network_json. Bad json\n");
            }
        }

        void apply_spike( const int spike_id, const double spike_time,
                const double spike_val=1, const bool normalize=true) 
        {
            try {

                _proc->apply_spike(Spike(_net->get_node(spike_id)->input_id,
                            spike_time, spike_val), normalize);

            } catch (const SRE &e) {
                printf(">>>>>>>> %s\n", e.what());
                exit(0);
            }   
        }

        void run(const double time)
        {
            _proc->run(time);
        }

        vector<double> get_output_vector()
        {
            return _proc->output_vectors()[0];
        }

        vector<int> get_neuron_counts()
        {
            return _proc->neuron_counts(0);
        }

        string make_viz_message(const vector<int> counts)
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
                        _net->sorted_node_vector[i]->id, i==n-1 ? "]" : ", ");
                msg += tmp;
            }

            return msg + "}\n"; 
        }

        double value_to_spike_time(const double value)
        {
            return round(_max_spike_time * (1 - value) / 2);
        }
};
