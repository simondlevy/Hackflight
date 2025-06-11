/**
 * Custom physics plugin for Hackflight simulator using ground-truth state 
 * and Spiking Neural Net controllers
 *
 * Adapted from
 * https://github.com/TENNLab-UTK/framework-open/blob/main/src/processor_tool.cpp
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 *
 */

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <unordered_set>
#include <unistd.h>

#include <framework.hpp>
#include <utils/json_helpers.hpp>

using namespace std;
using namespace neuro;
using nlohmann::json;

typedef runtime_error SRE;

static string node_name(Node *n) {
    if (n->name == "") return std::to_string(n->id);
    return (std::to_string(n->id)) + "(" + n->name + ")";
}

static int max_node_name_len(Network *net) 
{
    size_t i;
    Node *n;
    int max_name_len = 0;
    for (i = 0; i < net->sorted_node_vector.size(); i++) {
        n = net->sorted_node_vector[i];
        max_name_len = std::max(max_name_len, (int) node_name(n).size());
    }

    return max_name_len;

}

static void to_uppercase(string &s) 
{
    size_t i;
    for (i = 0; i < s.size(); i++) {
        if (s[i] >= 'a' && s[i] <= 'z') {
            s[i] = s[i] + 'A' -'a';
        }
    }
}

static int node_validation(const Network *n, const string &node_id)
{
    uint32_t nid;

    if (sscanf(node_id.c_str(), "%u", &nid) == 0) {
        throw SRE((string) "Bad node specification - " + node_id);
    }
    if (!n->is_node(nid)) throw SRE(node_id + " is not a node in the network");
    return nid;
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

static void output_node_id_validation(const int node_id, const Network *n) 
{
    Node *node;
    char buf[20];

    try {
        if (node_id < 0) throw "node_id must > 0";
        node = n->get_node(node_id);
        if (!node->is_output()) {
            snprintf(buf, 20, "%d", node_id);
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

static bool read_json(const vector <string> &sv, size_t starting_field, json &rv)
{
    bool success;
    string s;
    ifstream fin;

    rv.clear();
    if (starting_field < sv.size()) {
        fin.clear();
        fin.open(sv[starting_field].c_str());
        if (fin.fail()) { 
            perror(sv[starting_field].c_str());
            return false;
        } 
        try { fin >> rv; success = true; } catch(...) { success = false; }
        fin.close();
        return success;

    } else {
        try {
            cin >> rv;
            getline(cin, s);
            return true;
        } catch (...) {
            return false;
        }
    }
}

static Network *load_network(Processor **pp,
        const json &network_json)
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


static void safe_exit(Processor *p, Network *n)
{
    if (p != nullptr) delete p;
    if (n != nullptr) delete n;
    exit(0);
}

int main(int argc, char **argv) 
{
    Processor *p;
    Network *net, *pulled; 
    NodeMap::iterator nit;
    Node *node, *n;
    char buf[50];

    string proc_name, prompt;
    string cmd;
    string l,s;

    ofstream fout;

    istringstream ss;

    size_t i, j;
    int k;
    int node_id, output_id, spike_id, from, to;
    int max_name_len;
    double spike_time, spike_val;
    double sim_time;
    double val;
    string alias, id;

    vector <string> sv; // read inputs
    vector <Node *> node_vector;
    vector <Spike> spikes_array;
    vector <Spike> spikes;
    vector <double> output_times; // hold return value of output_vector()
    vector < vector <double> > all_output_times; // hold return value of output_vectors()
    vector < vector< double> > neuron_times;     // hold the return value of neuron_times();
    vector <string> spike_strings;              // hold spike strings from neuron_vectors_to_json()
    vector <int> v;
    vector <int> neuron_alias;
    vector <int> event_counts;
    vector <uint32_t> pres, posts;
    vector <double> weights;
    vector <double> charges;
    vector <double> data;
    vector <char> sr;
    map <int, double>::iterator mit;
    map <int, string> aliases; // Aliases for input/output nodes.
    map <int, string>::iterator ait;
    bool gsr_hidden_nodes;
    bool normalize;
    unordered_set <int> gsr_nodes;

    json proc_params, network_json;
    json spike_counts, spike_raster;
    json associated_data;
    json j1, j2;

    if (argc > 2 || (argc == 2 && strcmp(argv[1], "--help") == 0)) {
        fprintf(stderr, "usage: processor_tool [prompt]\n");
        fprintf(stderr, "\n");
        exit(1);
    }

    if (argc == 2) {
        prompt = argv[1];
        prompt += " ";
    }

    p = nullptr;
    net = nullptr;
    max_name_len = 0;

    while(1) {

        try {
            if (prompt != "") printf("%s", prompt.c_str());
            if (!getline(cin, l)) safe_exit(p, net);
            sv.clear();
            ss.clear();
            ss.str(l);

            while (ss >> s) sv.push_back(s);

            if (sv[0] == "ML") { // make() and load_network()

                if (!read_json(sv, 1, network_json)) {

                    printf("usage: ML network_json. Bad json\n");
                } else {

                    try {

                        if (p != nullptr) { delete p; p = nullptr; }
                        if (net != nullptr) { delete net; net = nullptr; }

                        net = load_network(&p, network_json);
                        max_name_len = max_node_name_len(net);

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

                else if (sv[0] == "AS" || sv[0] == "ASV") { // apply_spike()

                    if (network_processor_validation(net, p)) {
                        if (sv.size() < 2 || (sv.size() - 1) % 3 != 0) {
                            printf("usage: %s node_id spike_time spike_value node_id1 spike_time1 spike_value1 ...\n", sv[0].c_str());
                        } else {

                            normalize = (sv[0].size() == 2);
                            for (i = 0; i < (sv.size() - 1) / 3; i++) {
                                try {

                                    if (sscanf(sv[i*3 + 1].c_str(), "%d", &spike_id) != 1 ||
                                            sscanf(sv[i*3 + 2].c_str(), "%lf", &spike_time) != 1 || 
                                            sscanf(sv[i*3 + 3].c_str(), "%lf", &spike_val) != 1 ) {

                                        throw SRE((string) "Invalid spike [ " + sv[i*3 + 1] + "," + sv[i*3 + 2] + "," +
                                                sv[i*3 + 3] + "]\n");
                                    } 
                                    spike_validation(Spike(spike_id, spike_time, spike_val), net, normalize);

                                    p->apply_spike(Spike(net->get_node(spike_id)->input_id, spike_time, spike_val), normalize);
                                    spikes_array.push_back(Spike(spike_id, spike_time, spike_val));

                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                }   

                            }
                        }
                    }
                } else if (sv[0] == "ASR") { // apply_spike_raster()

                    if (network_processor_validation(net, p)) {
                        if (sv.size() != 3) {
                            printf("usage: ASR node_id spike_raster_string\n");
                        } else {

                            try {
                                sr.clear();
                                for (i = 0; i < sv[2].size(); i++) {
                                    if (sv[2][i] != '0' && sv[2][i] != '1') {
                                        throw SRE("ASR -- Spike raster string must be only 0's and 1's.");
                                    }
                                    sr.push_back(sv[2][i]-'0');
                                }
                                if (sscanf(sv[1].c_str(), "%d", &spike_id) != 1) {
                                    throw SRE((string) "Bad neuron id: " + sv[1]);
                                }
                                spike_validation(Spike(spike_id, 0, 0), net, true);
                                apply_spike_raster(p, net->get_node(spike_id)->input_id, sr);

                            } catch (const SRE &e) {
                                printf("%s\n",e.what());
                            }   
                        }
                    }

                } else if (sv[0] == "PS") {
                    for (i = 0; i < spikes_array.size(); i++){
                        printf("Spike: [%d,%lg,%lg]\n", spikes_array[i].id, spikes_array[i].time, spikes_array[i].value);
                    }

                } else if (sv[0] == "RUN") {

                    if (network_processor_validation(net, p)) {
                        if (sv.size() != 2 || sscanf(sv[1].c_str(), "%lf", &sim_time) != 1 || sim_time < 0) {
                            printf("usage: RUN sim_time. sim_time >= 0\n");
                        } else {

                            p->run(sim_time);
                            spikes_array.clear();

                        }
                    }

                } else if (sv[0] == "RUN_SR_CH" || sv[0] == "RSC") {

                    if (network_processor_validation(net, p)) {
                        if (sv.size() == 1 || sscanf(sv[1].c_str(), "%lf", &sim_time) != 1 || sim_time < 0) {
                            printf("usage: RSC/RUN_SR_CH sim_time [node] [...]\n");
                        } else {
                            spikes_array.clear();
                            net->make_sorted_node_vector();
                            gsr_nodes.clear();
                            for (i = 2 ; i < sv.size(); i++) gsr_nodes.insert(atoi(sv[i].c_str()));
                            if (gsr_nodes.empty()) {
                                for (i = 0; i < net->sorted_node_vector.size(); i++) {
                                    gsr_nodes.insert(net->sorted_node_vector[i]->id);
                                }
                            }

                            j1 = run_and_track(sim_time, p);

                            // Print the names of the nodes, twice.
                            printf("Time");
                            for (i = 0; i < net->sorted_node_vector.size(); i++) {
                                n = net->sorted_node_vector[i];
                                if (gsr_nodes.find(n->id) != gsr_nodes.end()) {
                                    printf(" %*s", max_name_len, node_name(n).c_str());
                                }
                            }
                            printf(" |");
                            for (i = 0; i < net->sorted_node_vector.size(); i++) {
                                n = net->sorted_node_vector[i];
                                if (gsr_nodes.find(n->id) != gsr_nodes.end()) {
                                    printf(" %*s", max_name_len, node_name(n).c_str());
                                }
                            }
                            printf("\n");

                            // Print spike raster and charge information for each timestep

                            for (i = 0; i < j1["spike_raster"].size(); i++) {
                                printf("%4d", (int) i);
                                for (j = 0; j < j1["spike_raster"][i].size(); j++) {
                                    n = net->sorted_node_vector[j];
                                    if (gsr_nodes.find(n->id) != gsr_nodes.end()) {
                                        k = j1["spike_raster"][i][j];
                                        printf(" %*c", max_name_len, (k == 1) ? '*' : '-');
                                    }
                                }
                                printf(" |");
                                for (j = 0; j < j1["charges"][i].size(); j++) {
                                    n = net->sorted_node_vector[j];
                                    if (gsr_nodes.find(n->id) != gsr_nodes.end()) {
                                        val = j1["charges"][i][j];
                                        printf(" %*lg", max_name_len, val);
                                    }
                                }
                                printf("\n");
                            }
                        }
                    }

                } else if (sv[0] == "GT") { // get_time()

                    /* the behavior of Caspian and GNP is different. 
                       Caspian will get the sum to running time. GNP will get the simulation time
                     */
                    if (network_processor_validation(net, p)) 
                        printf("time: %.1lf\n", p->get_time());

                } else if (sv[0] == "NLF") { // test neuron_last_fires

                    if (sv.size() == 1 || (sv[1] != "T" && sv[1] != "F")) {
                        printf("usage: NLF show_nonfiring - Last fire times for all neurons. show_nonfiring=T/F\n");
                    } else if (network_processor_validation(net, p)) {
                        net->make_sorted_node_vector();
                        output_times = p->neuron_last_fires();
                        if (output_times.size() == 0) {
                            printf("Recording last fire times for neurons is not implemented by %s.\n",
                                    p->get_name().c_str());
                        }

                        for (i = 0; i < output_times.size(); i++) {
                            node = net->sorted_node_vector[i];
                            node_id = node->id;
                            if (output_times[i] != -1.0 || sv[1][0] == 'T') {
                                printf("Node %*s last fire: %.1lf\n", max_name_len, node_name(node).c_str(), output_times[i]);
                            }
                        }
                    }

                } else if (sv[0] == "GT") { // get_time()

                    /* the behavior of Caspian and GNP is different. 
                       Caspian will get the sum to running time. GNP will get the simulation time
                     */
                    if (network_processor_validation(net, p)) 
                        printf("time: %.1lf\n", p->get_time());

                } else if (sv[0] == "NLF") { // test neuron_last_fires

                    if (sv.size() == 1 || (sv[1] != "T" && sv[1] != "F")) {
                        printf("usage: NLF show_nonfiring - Last fire times for all neurons. show_nonfiring=T/F\n");
                    } else if (network_processor_validation(net, p)) {
                        net->make_sorted_node_vector();
                        output_times = p->neuron_last_fires();
                        if (output_times.size() == 0) {
                            printf("Recording last fire times for neurons is not implemented by %s.\n",
                                    p->get_name().c_str());
                        }

                        for (i = 0; i < output_times.size(); i++) {
                            node = net->sorted_node_vector[i];
                            node_id = node->id;
                            if (output_times[i] != -1.0 || sv[1][0] == 'T') {
                                printf("Node %*s last fire: %.1lf\n", max_name_len, node_name(node).c_str(), output_times[i]);
                            }
                        }
                    }

                } else if (sv[0] == "TNC") { // test total_neuron_counts
                    if (sv.size() != 1) {
                        printf("usage: TNC - Total fire counts for all neurons.\n");
                    } else if (network_processor_validation(net, p)) {
                        printf("%lld\n", p->total_neuron_counts());
                    }

                } else if (sv[0] == "TNA") { // test total_neuron_counts
                    if (sv.size() != 1) {
                        printf("usage: TNA - Total accumulates for all neurons.\n");
                    } else if (network_processor_validation(net, p)) {
                        printf("%lld\n", p->total_neuron_accumulates());
                    }

                } else if (sv[0] == "NC") { // test neuron_counts

                    if (sv.size() == 1 || (sv[1] != "T" && sv[1] != "F")) {
                        printf("usage: NC show_nonfiring - Fire counts for all neurons. show_nonfiring=T/F\n");
                    } else if (network_processor_validation(net, p)) {
                        net->make_sorted_node_vector();
                        event_counts = p->neuron_counts();
                        if (event_counts.size() == 0) {
                            printf("Recording event counts for neurons is not implemented by %s.\n",
                                    p->get_name().c_str());
                        }


                        for (i = 0; i < event_counts.size(); i++) {
                            node = net->sorted_node_vector[i];
                            node_id = node->id;
                            if (event_counts[i] != 0 || sv[1][0] == 'T') {
                                printf("Node %*s fire count: %d\n", max_name_len, node_name(node).c_str(), event_counts[i]);
                            }
                        }
                    }

                } else if (sv[0] == "NCH") { // test neuron_charges

                    if (network_processor_validation(net, p)) {
                        net->make_sorted_node_vector();
                        charges = p->neuron_charges();
                        if (charges.size() == 0) {
                            printf("Recording charges for neurons is not implemented by %s.\n",
                                    p->get_name().c_str());
                        }

                        v.clear();
                        for (i = 1; i < sv.size(); i++) {
                            v.push_back(node_validation(net, sv[i]));
                        }
                        sort(v.begin(), v.end());
                        j = 0;
                        for (i = 0; i < charges.size(); i++) {
                            node = net->sorted_node_vector[i];
                            node_id = net->sorted_node_vector[i]->id;
                            if (v.size() == 0 || (j < v.size() && v[j] == node_id)) {
                                printf("Node %*s charge: %lg\n", max_name_len, node_name(node).c_str(), charges[i]);
                                j++;
                            }
                        }
                    }

                } else if (sv[0] == "SW") { // test synapse_weights

                    if (sv.size() != 1 && sv.size() != 3) {
                        printf("usage: SW [from to] - show weights for all synapses, or just one synapse.\n");
                    } else if (network_processor_validation(net, p)) {
                        if (sv.size() == 3) {
                            from = atoi(sv[1].c_str());
                            to = atoi(sv[2].c_str());
                        } else {
                            from = -1;
                        }
                        p->synapse_weights(pres, posts, weights);

                        /* Since the previous call is O(synapses anyway), I'm not embarrased of
                           this loop when from/to are specified. */

                        for (i = 0; i < pres.size(); i++) {
                            if (from == -1 || (from == (int) pres[i] && to == (int) posts[i])) {
                                printf("  %4u -> %4u : %7.4lf\n", pres[i], posts[i], weights[i]);
                            }
                        }
                    }

                } else if (sv[0] == "PULL_NETWORK") { // test pull_network

                    if (sv.size() != 2) {
                        printf("usage: PULL_NETWORK file - pull network from the processor to the file.\n");
                    } else if (network_processor_validation(net, p)) {
                        fout.clear();
                        fout.open(sv[1]);
                        if (fout.fail()) {
                            perror(sv[1].c_str());
                        } else {
                            pulled = pull_network(p, net);
                            fout << pulled->as_json() << endl;
                            fout.close();
                            delete pulled;
                        }         
                    }

                } else if (sv[0] == "NV" || sv[0] == "NT") { // test neuron_vectors

                    if (sv.size() == 1 || (sv[1] != "T" && sv[1] != "F")) {
                        printf("usage: NV show_nonfiring - Fire counts for all neurons. show_nonfiring=T/F\n");
                    } else if (network_processor_validation(net, p)) {
                        net->make_sorted_node_vector();
                        try {
                            neuron_times = p->neuron_vectors();
                            if (neuron_times.size() == 0) {
                                printf("Recording events for neurons is not implemented by %s.\n",
                                        p->get_name().c_str());
                            } else {
                                for (i = 0; i < net->sorted_node_vector.size(); i++) {
                                    node_id = net->sorted_node_vector[i]->id;
                                    if (neuron_times[i].size() > 0 || sv[1][0] == 'T') {
                                        printf("Node %2u fire times:", node_id);

                                        for (j = 0; j < neuron_times[i].size(); j++) {
                                            printf(" %.1lf", neuron_times[i][j]);
                                        }
                                        printf("\n");
                                    }
                                }
                            }
                        } catch (const SRE &e) {
                            printf("%s\n",e.what());
                        } catch (...) {
                            printf("Unknown error\n");
                        }
                    }

                } else if (sv[0] == "OLF") { // test output_last_fire and output_last_fires.

                    if (network_processor_validation(net, p)) {

                        if (sv.size() == 1) {
                            output_times = p->output_last_fires();
                            for (i = 0; i < (size_t)net->num_outputs(); i++) {
                                node = net->get_output(i);
                                printf("node %s last fire time: %.1lf\n", node_name(node).c_str(), output_times[i]);
                            }
                        } else {

                            for (i = 1; i < sv.size(); i++) {
                                try {
                                    if (sscanf(sv[i].c_str(), "%d", &node_id) != 1) {
                                        throw SRE(sv[i] + " is not a valid node id");
                                    }
                                    output_node_id_validation(node_id, net);
                                    output_id = net->get_node(node_id)->output_id;
                                    node = net->get_node(node_id);
                                    printf("node %s last fire time: %.1lf\n", node_name(node).c_str(), p->output_last_fire(output_id));

                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                }
                            }
                        }
                    }

                } else if (sv[0] == "OC") {   // Test output_count and output_counts
                    if (network_processor_validation(net, p)) {
                        if (sv.size() == 1) {
                            event_counts = p->output_counts();
                            for (i = 0; i < (size_t)net->num_outputs(); i++) {
                                node = net->get_output(i);
                                printf("node %s spike counts: %d\n", node_name(node).c_str(), event_counts[i]);
                            }
                        } else {

                            for (i = 1; i < sv.size(); i++) {
                                try {
                                    if (sscanf(sv[i].c_str(), "%d", &node_id) != 1) {
                                        throw SRE(sv[i] + " is not a valid node id");
                                    }
                                    output_node_id_validation(node_id, net);

                                    output_id = net->get_node(node_id)->output_id;
                                    node = net->get_node(node_id);
                                    printf("node %s spike counts: %d\n", node_name(node).c_str(), p->output_count(output_id));
                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                }
                            }
                        }
                    }

                } else if (sv[0] == "TRACK_O" || sv[0] == "UNTRACK_O") { // track_output_events() and track_all_output_events()
                    if (network_processor_validation(net, p)) {
                        if (sv.size() == 1) {
                            if (sv[0][0] == 'T') {
                                (void) track_all_output_events(p, net);
                            } else {
                                for (i = 0; i < (size_t)net->num_outputs(); i++) p->track_output_events(i, false);
                            } 
                        } else {
                            for (i = 1; i < sv.size(); i++) {
                                try {
                                    if (sscanf(sv[i].c_str(), "%d", &node_id) != 1) {
                                        throw SRE(sv[i] + " is not a valid node id");
                                    }
                                    output_node_id_validation(node_id, net);

                                    output_id = net->get_node(node_id)->output_id;
                                    if (!p->track_output_events(output_id, (sv[0][0] == 'T'))) {
                                        snprintf(buf, 50, "%d, %d) failed.", output_id, (sv[0][0] == 'T'));
                                        throw SRE((string) "track_output_events(" + buf);
                                    }
                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                }
                            }
                        } 

                    }
                } else if (sv[0] == "TRACK_N" || sv[0] == "UNTRACK_N") { // track_neuron_events() and track_all_neuron_events()
                    if (network_processor_validation(net, p)) {
                        if (sv.size() == 1) {
                            if (sv[0][0] == 'T') {
                                if (!track_all_neuron_events(p, net)) {
                                    printf("track_all_neuron_events() not supported by processor.\n");
                                }
                            } else {
                                for (nit = net->begin(); nit != net->end(); nit++) {
                                    p->track_neuron_events(nit->second->id, false);
                                }
                            } 
                        } else {
                            for (i = 1; i < sv.size(); i++) {
                                try {
                                    if (sscanf(sv[i].c_str(), "%d", &node_id) != 1 || node_id < 0) {
                                        throw SRE(sv[i] + " is not a valid node id");
                                    }
                                    if (!p->track_neuron_events(node_id, (sv[0][0] == 'T'))) {
                                        snprintf(buf, 50, "%d, %d) failed.", node_id, (sv[0][0] == 'T'));
                                        throw SRE((string) "track_neuron_events(" + buf);
                                    }
                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                }
                            }
                        } 

                    }

                } else if (sv[0] == "OT" || sv[0] == "OV") {  //  output_vector(int output_id, int network_id = 0)
                    if (network_processor_validation(net, p)) {

                        /* if OT doesn't take any node_ids, do all outputs */

                        if (sv.size() == 1) {
                            try {
                                all_output_times = p->output_vectors();
                                if (all_output_times.size() == 0) {
                                    throw SRE("Processor error -- p->output_vectors returned a vector of size zero");
                                } 
                                for (i = 0; i < (size_t)net->num_outputs(); i++) {

                                    node = net->get_output(i);
                                    printf("node %s spike times:", node_name(node).c_str());
                                    for (j = 0; j < all_output_times[i].size(); j++) {
                                        printf(" %.1lf", all_output_times[i][j]);
                                    }
                                    printf("\n");
                                }
                            } catch (const SRE &e) {
                                printf("%s\n",e.what());
                            } catch (...) {
                                printf("Unknown error\n");
                            }
                        } else {

                            for (i = 1; i < sv.size(); i++) {
                                try {

                                    if (sscanf(sv[i].c_str(), "%d", &node_id) != 1) {
                                        throw SRE(sv[i] + " is not a valid node id");
                                    }
                                    output_node_id_validation(node_id, net);
                                    output_id = net->get_node(node_id)->output_id;

                                    output_times = p->output_vector(output_id);
                                    node = net->get_node(node_id);
                                    printf("node %s spike times: ", node_name(node).c_str());
                                    for (j = 0; j < output_times.size(); j++) {
                                        printf("%.1lf ",output_times[j]);
                                    }
                                    printf("\n");

                                } catch (const SRE &e) {
                                    printf("%s\n",e.what());
                                } catch (...) {
                                    printf("Unknown error\n");
                                }
                            }
                        }
                    }


                } else if (sv[0] == "CA" || sv[0] == "CLEAR-A") { // clear_activity

                    if (network_processor_validation(net, p)) 
                        p->clear_activity();

                } else if (sv[0] == "CLEAR" || sv[0] == "C") {

                    if (network_processor_validation(net, p)) {
                        p->clear();
                        delete net;
                        net = nullptr;
                    }


                } else if (sv[0] == "PP") { // get_processor_properties
                    if (p == nullptr) {
                        printf("Must make a processor first\n"); 
                    } else {
                        cout << p->get_processor_properties() << endl;
                    } 

                } else if (sv[0] == "NP" || sv[0] == "PPACK") { // get_network_properties
                    if (p == nullptr) {
                        printf("Must make a processor first\n"); 
                    } else {
                        cout << p->get_network_properties().pretty_json() << endl;
                    } 

                } else if (sv[0] == "NAME") { 
                    if (p == nullptr) {
                        printf("Must make a processor first\n"); 
                    } else {
                        cout << p->get_name() << endl;
                    } 

                } else if (sv[0] == "EMPTYNET") { 
                    if (p == nullptr) {
                        printf("Must make a processor first\n"); 
                    } else {
                        if (sv.size() > 1) {
                            fout.clear();
                            fout.open(sv[1].c_str());
                            if (fout.fail()) {
                                perror(sv[1].c_str());
                                throw SRE("");
                            } 
                        }
                        if (net != nullptr) { delete net; net = nullptr; }
                        net = new Network;
                        net->set_properties(p->get_network_properties());
                        j1 = json::object();
                        j1["proc_name"] = p->get_name();
                        net->set_data("other", j1);
                        net->set_data("proc_params", p->get_params());
                        if (sv.size() == 1) {
                            cout << net->pretty_json() << endl;
                        } else {
                            fout << net->as_json() << endl;
                            fout.close();
                        }
                    }
                } else if (sv[0] == "PARAMS") { 
                    if (sv.size() != 1 && sv.size() != 2) {
                        cout << "usage: PARAMS [file] " << endl;
                    } else if (p == nullptr) {
                        printf("Must make a processor first\n"); 
                    } else if (sv.size() == 2) {
                        fout.clear();
                        fout.open(sv[1].c_str());
                        if (fout.fail()) {
                            perror(sv[1].c_str());
                        } else {
                            fout << p->get_params() << endl;
                            fout.close();
                        }
                    } else {
                        cout << p->get_params().dump(2) << endl;
                    }

                } else if (sv[0] == "GSR") { // get_spike_raster
                    gsr_nodes.clear();
                    gsr_hidden_nodes = true;
                    if (sv.size() > 1) {
                        if (sv[1] == "T" || sv[1] == "F") {
                            gsr_hidden_nodes = (sv[1] == "T");
                            i = 2;
                        } else {
                            i = 1;
                        }
                        for ( ; i < sv.size(); i++) {
                            gsr_nodes.insert(atoi(sv[i].c_str()));
                        }
                    }
                    if (network_processor_validation(net, p)) {

                        net->make_sorted_node_vector();
                        neuron_times = p->neuron_vectors();
                        spike_raster = neuron_vectors_to_json(neuron_times, "S", net);
                        spike_strings = spike_raster["Spikes"].get<vector <string>>();

                        for(i = 0; i < net->sorted_node_vector.size(); i++){
                            node = net->sorted_node_vector[i];
                            if (gsr_nodes.size() == 0 || gsr_nodes.find(node->id) != gsr_nodes.end()) {
                                if (gsr_hidden_nodes || !node->is_hidden()) {
                                    if(node->is_input()) {
                                        id = "INPUT ";
                                    } else if (node->is_output()) {
                                        id = "OUTPUT";
                                    } else {
                                        id = "HIDDEN";
                                    }
                                    printf("%-*s %s : %s\n", max_name_len, node_name(node).c_str(), 
                                            id.c_str(), spike_strings[i].c_str());
                                }
                            }
                        }
                    }


                } 

            } catch (const SRE &e) {
                printf("%s\n", e.what());
            }
        }  // end of while loop
    }
