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

#include <framework.hpp>
#include <utils/json_helpers.hpp>

using namespace std;
using namespace neuro;
using nlohmann::json;

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

static Network *load_network(Processor **pp, const json &network_json)
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
        throw SRE("load_network() failed");
    }
    track_all_neuron_events(p, net);

    return net;
}

static void apply_spike(
        Network * net,
        Processor *p,
        const int spike_id,
        const double spike_time,
        vector<Spike> & spikes_array,
        const double spike_val=1,
        const bool normalize=true) 
{
    try {

        p->apply_spike(Spike(net->get_node(spike_id)->input_id,
                    spike_time, spike_val), normalize);

        spikes_array.push_back(Spike(spike_id, spike_time, spike_val));

    } catch (const SRE &e) {
        printf("%s\n",e.what());
    }   
}

static double get_spike_time(const float inp, const double max)
{
    return round(max * (1 - inp) / 2);
}

static Network * load_json(const char * network_filename, Processor ** proc)
{
    Network * net = nullptr;

    json network_json = {};

    if (!read_json(network_filename, network_json)) {

        printf("usage: ML network_json. Bad json\n");

    } else {

        try {

            net = load_network(proc, network_json);

        } catch (const SRE &e) {
            printf("%s\n",e.what());
        } catch (...) {
            printf("Unknown error when making processor\n");
        }
    }

    return net;
}
