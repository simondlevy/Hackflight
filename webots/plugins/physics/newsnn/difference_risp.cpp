#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <unordered_set>
#include <unistd.h>
#include <math.h>

#include "framework.hpp"
#include "utils/json_helpers.hpp"

using namespace std;
using namespace neuro;
using nlohmann::json;

typedef runtime_error SRE;

static bool network_processor_validation(const Network *n, const Processor *p) {
    bool success = (n != nullptr && p != nullptr);

    if (!success) {
        printf("Processor or network is not loaded\n");
    }
    return success;
}

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

    if (p->get_network_properties().as_json() != net->get_properties().as_json()) {
        throw SRE("network and processor properties do not match.");
    }

    if (!p->load_network(net)) throw SRE("load_network() failed");
    track_all_neuron_events(p, net);

    return net;
}

static void apply_spike(
        Network * net,
        Processor *p,
        const int spike_id,
        const double spike_time,
        vector<Spike> & spikes_array) 
{
    static const double SPIKE_VAL = 1;

    static const bool NORMALIZE = true;

    if (network_processor_validation(net, p)) {

        try {

            p->apply_spike(Spike(net->get_node(spike_id)->input_id, spike_time, SPIKE_VAL), NORMALIZE);

            spikes_array.push_back(Spike(spike_id, spike_time, SPIKE_VAL));

        } catch (const SRE &e) {
            printf("%s\n",e.what());
        }   
    }
}

static double get_spike_time(const float inp, const double max)
{
    return round(max * (1 - inp) / 2);
}

int main() 
{
    static const char * NETWORK_FILENAME = "/home/levys/Desktop/diffnet/difference_risp_plank.txt";

    static const double MAX = 1000;

    Network * net = nullptr;

    Processor * proc = nullptr;

    // Load network ----------------------------------------------------------

    json network_json = {};

    if (!read_json(NETWORK_FILENAME, network_json)) {

        printf("usage: ML network_json. Bad json\n");
        exit(1);

    } else {

        try {

            net = load_network(&proc, network_json);

        } catch (const SRE &e) {
            printf("%s\n",e.what());
            exit(1);
        } catch (...) {
            printf("Unknown error when making processor\n");
            exit(1);
        }
    }

    for (int j=-10; j<=+10; ++j) {

        for (int k=-10; k<+10; ++k) {

            const double inp1 = (double)j / 10;
            const double inp2 = (double)k / 10;

            const double spike_time_1 = get_spike_time(inp1, MAX);
            const double spike_time_2 = get_spike_time(inp2, MAX);

            vector <Spike> spikes_array = {};

            apply_spike(net, proc, 0, spike_time_1, spikes_array);
            apply_spike(net, proc, 1, spike_time_2, spikes_array);
            apply_spike(net, proc, 2, 0, spikes_array);

            if (network_processor_validation(net, proc)) {

                const auto sim_time = 3 * MAX + 2;

                proc->run(sim_time);

                spikes_array.clear();
            }

            const auto out = proc->output_vectors()[0][0];

            const auto time = out == MAX + 1 ? -2 : out;

            const auto diff = (time-(MAX))*4/(2*MAX)-2;

            printf("%+3.3f - %+3.3f = %+3.3f (%+3.3f)\n",
                    inp1, inp2, diff, inp1-inp2);
        }
    }
}
