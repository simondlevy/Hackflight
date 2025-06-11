#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <unordered_set>
#include <unistd.h>
#include "framework.hpp"
#include "utils/json_helpers.hpp"

using namespace std;
using namespace neuro;
using nlohmann::json;

typedef runtime_error SRE;

static string node_name(Node *n) 
{
    if (n->name == "") return std::to_string(n->id);
    return (std::to_string(n->id)) + "(" + n->name + ")";
}

static void spike_validation(const Spike &s, const Network *n, bool normalize) 
{
    Node *node;
    char buf[20];

    try {
        if (normalize) {
            if (s.value < -1 || s.value > 1) throw "spike val must be >= -1 and <= 1";
        }
        if (s.time < 0) throw "spike time must be > 0";
        node = n->get_node(s.id);
        if (!node->is_input()) {
            snprintf(buf, 20, "%d", s.id);
            throw (string) "node " + buf + " is not an input node";
        }

    } catch (const string &s) {
        throw SRE(s);
    } catch (const char *s) {
        throw SRE(s);
    }

}

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

static void apply_spike(Network * net, Processor *p, vector<string> & sv,
        vector<Spike> & spikes_array) 
{
    static const double SPIKE_VAL = 1;

    static const bool NORMALIZE = true;

    if (network_processor_validation(net, p)) {

        //for (size_t i = 0; i < (sv.size() - 1) / 3; i++) {

        //    fprintf(stderr, "i=%lu\n", i);

            try {

                int spike_id = 0;
                double spike_time = 0;

                sscanf(sv[1].c_str(),"%d", &spike_id);
                sscanf(sv[2].c_str(), "%lf", &spike_time);

                spike_validation(Spike(spike_id, spike_time, SPIKE_VAL), net, NORMALIZE);

                p->apply_spike(Spike(net->get_node(spike_id)->input_id, spike_time, SPIKE_VAL), NORMALIZE);

                spikes_array.push_back(Spike(spike_id, spike_time, SPIKE_VAL));

            } catch (const SRE &e) {
                printf("%s\n",e.what());
            }   

        //}
    }
}

int main(int argc, char **argv) 
{
    static const char * NETWORK_FILENAME = "difference_risp_plank.txt";
    static const double MAX = 1000;

    istringstream ss = {};

    vector <string> sv = {}; 

    vector <Spike> spikes_array = {};

    Network * net = nullptr;

    Processor * p = nullptr;

    if (argc > 2 || (argc == 2 && strcmp(argv[1], "--help") == 0)) {
        fprintf(stderr, "usage: processor_tool [prompt]\n");
        fprintf(stderr, "\n");
        exit(1);
    }

    string prompt = {};

    if (argc == 2) {
        prompt = argv[1];
        prompt += " ";
    }

    // Load network ----------------------------------------------------------

    json network_json = {};

    if (!read_json(NETWORK_FILENAME, network_json)) {

        printf("usage: ML network_json. Bad json\n");
    } else {

        try {

            if (p != nullptr) { delete p; p = nullptr; }
            if (net != nullptr) { delete net; net = nullptr; }

            net = load_network(&p, network_json);

        } catch (const SRE &e) {
            printf("%s\n",e.what());
            if (net != nullptr) { delete net; net = nullptr; }
            if (p != nullptr) { delete p; p = nullptr; }
            net = nullptr;
            p = nullptr;
        } catch (...) {
            printf("Unknown error when making processor\n");
            if (net != nullptr) { delete net; net = nullptr; }
            if (p != nullptr) { delete p; p = nullptr; }
            net = nullptr;
            p = nullptr;

        }
    }

    while(true) {

        try {

            if (prompt != "") printf("%s", prompt.c_str());

            string l = {};
            if (!getline(cin, l)) break; 
            sv.clear();
            ss.clear();
            ss.str(l);

            string s = {};
            while (ss >> s) sv.push_back(s);

            if (sv[0] == "AS") { 
                apply_spike(net, p, sv, spikes_array);
            } 

        } catch (const SRE &e) {
            printf("%s\n", e.what());
        }
    }  

    // Run -------------------------------------------------------------------

    if (network_processor_validation(net, p)) {

        const auto sim_time = 3 * MAX + 2;

        p->run(sim_time);

        spikes_array.clear();
    }

    // Output -----------------------------------------------------------------

    const auto all_output_times = p->output_vectors();

    for (size_t i = 0; i < (size_t)net->num_outputs(); i++) {

        auto node = net->get_output(i);

        printf("node %s spike times:", node_name(node).c_str());

        for (size_t j=0; j<all_output_times[i].size(); j++) {
            printf(" %.1lf", all_output_times[i][j]);
        }
        printf("\n");
    }
}
