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

        static Network * loadnetwork(Processor **pp, const json &network_json)
        {
            Network *net;
            json proc_params;
            string proc_name;
            Processor *p;

            net = new Network();
            net->from_json(network_json);

            p = *pp;
            if (p == nullptr) {
                proc_params = net->get_data("proc_params");
                proc_name = net->get_data("other")["proc_name"];
                p = Processor::make(proc_name, proc_params);
                *pp = p;
            } 

            if (p->get_network_properties().as_json() !=
                    net->get_properties().as_json()) {
                throw SRE("network and processor properties do not match.");
            }

            if (!p->load_network(net)) {
                throw SRE("loadnetwork() failed");
            }
            track_all_neuron_events(p, net);

            return net;
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
            _net = Framework::load(network_filename, &_proc);
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

        //////////////////////////////////////////////////////////////////////

        static double value_to_spike_time(const double val, const double max)
        {
            return Framework::get_spike_time(cap(val), max);
        }

        static Network * load(const char * network_filename, Processor ** proc)
        {
            Network * net = nullptr;

            json network_json = {};

            if (!read_json(network_filename, network_json)) {

                printf("usage: ML network_json. Bad json\n");

            } else {

                try {

                    net = loadnetwork(proc, network_json);

                } catch (const SRE &e) {
                    printf("%s\n",e.what());
                } catch (...) {
                    printf("Unknown error when making processor\n");
                }
            }

            return net;
        }

        static double get_spike_time(const double inp, const double max)
        {
            return round(max * (1 - inp) / 2);
        }

        static void apply_spike(
                Network * net,
                Processor *p,
                const int spike_id,
                const double spike_time,
                const double spike_val=1,
                const bool normalize=true) 
        {
            try {

                p->apply_spike(Spike(net->get_node(spike_id)->input_id,
                            spike_time, spike_val), normalize);

            } catch (const SRE &e) {
                printf(">>>>>>>> %s\n", e.what());
                exit(0);
            }   
        }


};
