#pragma once

#include <etl/unordered_map.h>

#include "framework.hpp"
#include "utils/MOA.hpp"

using namespace neuro;

namespace risp
{

    // Arbitrary array limits
    static const size_t MAX_EVENT_TIMES = 4000;
    static const size_t MAX_EVENTS_PER_TIME = 10;
    static const size_t MAX_SYNAPSES = 20;

    class Neuron {

        public:

            Neuron() = default;

            void init(uint32_t node_id, double t, bool l) 
            {
                charge = 0;
                threshold = t;
                last_check = -1;
                last_fire = -1;
                fire_counts = 0;
                leak = l;
                id = node_id;
                check = false; 
            }

            void perform_fire(int time)
            {
                last_fire = time;
                fire_counts++;
                charge = 0;
            }

            class Synapse * synapses[MAX_SYNAPSES];
            size_t n_synapses;

            double charge;               
            double threshold;            
            int last_check;              
            int last_fire;               
            uint32_t fire_counts;        
            bool leak;                   
            uint32_t id;                 
            bool check;                  
            bool track;                  
    };

    class Synapse {

        public:

            Synapse() = default;

            void init(double w, uint32_t d, Neuron* to_n)
            {
                weight = w;
                delay = d;
                to = to_n;
            }

            double weight;             
            Neuron *to;                
            uint32_t delay;            
    };

    class Network {

        public:

            Network() = default;

            void init(neuro::Network *net)
            {
                n_neurons = 0;
                n_synapses = 0;
                n_inputs = 0;
                overall_run_time = 0;
                neuron_fire_counter = 0;
                neuron_accum_counter = 0;

                rng.Seed(0, "noisy_risp");

                net->make_sorted_node_vector();

                // for(size_t i=0; i<net->sorted_node_vector.size(); i++) {
                for(auto node : net->sorted_node_vector) {

                    // No leak support at this point; should support 
                    // LEAK_ALL and LEAK_CONFIGURABLE;
                    const bool neuron_leak = false; 

                    Neuron * neuron = add_neuron(node->id, node->threshold,
                            neuron_leak);

                    if (node->is_input()) {
                        add_input(node->id, node->input_id);
                    }

                    if (node->is_output()) {
                        add_output(node->id, node->output_id);
                    }

                    sorted_neuron_vector.push_back(neuron);
                }

                // Add synpases
                for (size_t k=0; k<net->n_edges; ++k) {
                    neuro::Edge *edge = &net->edges[k];
                    add_synapse(edge->from->id, edge->to->id,
                            edge->weight, edge->delay);
                }
            }

            void apply_spike(const Spike& s, bool normalized) 
            {
                if (normalized && (s.value < 0 || s.value > 1)) {
                    printf("risp::Network::apply_spike() - value (%lg) must be "
                            "in [-1,1].\n", s.value);
                }

                if (!is_valid_input_id(s.id)) {
                    printf("risp::Network::apply_spike() - input_id %d "
                            "is not valid\n", s.id);
                }

                if (s.time >= events.size()) {
                    events.resize(s.time + 1);
                }

                double v = 0;

                if (normalized) {
                    v = true ?
                        floor(s.value * 1000.0) :
                        s.value * 1000.0;
                } else {
                    v = s.value;
                }

                if (0 != 0) {
                    v = rng.Random_Normal(v, 0);
                }

                auto n = get_neuron(inputs[s.id]);
                events[s.time].push_back(etl::make_pair(n,v));
            }

            void run(double duration) 
            {
                if (duration < 0) {
                    printf("risp::Network::run() - duration < 0\n");
                }

                if (overall_run_time != 0) {
                    clear_tracking_info();
                }

                int run_time = (false) ?  duration : duration-1;

                overall_run_time += (run_time+1);

                /* expand the event buffer */
                if ((int) events.size() <= run_time) {
                    events.resize(run_time + 1);
                }

                for (int i=0; i<=run_time; i++) {
                    process_events(i);
                }

                // shift extra spiking events to the beginning from the
                // previous run() call
                if ((int) events.size() > run_time + 1) {

                    for (size_t i = run_time + 1; i < events.size(); i++) {

                        // Google tells me this is O(1)
                        events[i - run_time - 1] = etl::move(events[i]);   
                    }
                }

                for (auto neuron : sorted_neuron_vector) {
                    if (neuron->leak) neuron->charge = 0;
                    if (neuron->charge < -1000.0) {
                        neuron->charge = -1000.0;
                    }
                }
            }

            bool track_neuron_events(uint32_t node_id, bool track) 
            {
                if(!is_neuron(node_id)) {
                    return false;
                }
                neuron_map[node_id]->track = track;

                return true;
            }

            etl::vector<double, 1> get_output_fire_times()
            {
                etl::vector<double, 1> times;

                for (size_t k=0; k<n_outputs; ++k) {
                    times.push_back(neuron_map[outputs[k]]->last_fire);
                }

                return times;
            }

            etl::vector<int, 7> get_neuron_counts() 
            {
                etl::vector<int, 7> counts;

                for (auto neuron : sorted_neuron_vector) {
                    counts.push_back(neuron->fire_counts);
                }

                return counts;
            }

                protected:

            size_t n_neurons;
            Neuron neurons[7];

            size_t n_synapses;
            Synapse synapses[MAX_SYNAPSES];

            int inputs[3];
            size_t n_inputs;

            int outputs[1];
            size_t n_outputs;

            etl::vector<Neuron *, 7> sorted_neuron_vector;         

            etl::unordered_map <uint32_t, Neuron *, 7> neuron_map;   

            etl::vector<
                etl::vector<
                etl::pair<Neuron *, double>,
                MAX_EVENTS_PER_TIME
                    > , MAX_EVENT_TIMES
                    > events;

            long long neuron_fire_counter;  
            long long neuron_accum_counter; 
            int overall_run_time;     
            MOA rng;
            etl::vector<Neuron *, 7> to_fire;   

            Neuron * add_neuron(uint32_t node_id, double threshold, bool leak) 
            {
                Neuron * n = nullptr;

                if (is_neuron(node_id)) {
                    printf("risp::Neuron::add_neuron() - %d is already "
                            "in the neuron map\n", (int)node_id);
                }

                if (n_neurons+1 == 7) {
                    printf("Network::add_neuron: max of %d neurons exceeded\n", 
                            (int)7);
                }

                else {

                    neurons[n_neurons].init(node_id, threshold, leak);

                    n = &neurons[n_neurons];

                    n_neurons++;

                    if (!true) {
                        n->threshold = (true) ?
                            (n->threshold+1) :(n->threshold + 0.0000001);
                    }

                    neuron_map[node_id] = n;
                }

                return n;
            }

            Neuron* get_neuron(uint32_t node_id) 
            {
                auto it = neuron_map.find(node_id);

                if (it == neuron_map.end()) {
                    printf("risp::Network::get_neuron() - %d is not in the "
                            "neuron map\n", (int)node_id);
                }
                return it->second;
            }

            bool is_neuron(uint32_t node_id) 
            {
                return neuron_map.find(node_id) != neuron_map.end();
            }

            bool is_valid_input_id(int input_id) 
            {
                return !(input_id < 0 || input_id >= (int)n_inputs);
            }

            bool is_valid_output_id(int output_id) 
            {
                return !(output_id < 0 || output_id >= (int) n_outputs ||
                        outputs[output_id] == -1);
            }

            Synapse* add_synapse(uint32_t from_id, uint32_t to_id,
                    double weight, uint32_t delay) 
            {
                Synapse * syn = nullptr;

                if (!is_neuron(from_id)) {
                    printf("risp::Network::add_synpase() - "
                            "node %d does not exist", (int)from_id);
                }

                if (!is_neuron(to_id)) {
                    printf("risp::Network::add_synpase() - "
                            "node %d does not exist", (int)to_id);
                }

                Neuron * from = get_neuron(from_id);
                Neuron * to = get_neuron(to_id);

                if (n_synapses+1 == MAX_SYNAPSES) {
                    printf("Network::add_synapse: max of %d synapses "
                            "exceeded\n", (int)MAX_SYNAPSES);
                }

                else {

                    synapses[n_synapses].init(weight, delay, to);
                    syn = &synapses[n_synapses];
                    from->synapses[from->n_synapses++] = syn;
                    n_synapses++;
                }

                return syn;
            }


            void add_input(uint32_t node_id, int input_id) 
            {
                if (!is_neuron(node_id)) {
                    printf("risp::Network::add_input() - node %d "
                            "does not exist", (int)node_id);
                }
                if (input_id < 0) {
                    printf("risp::Network::add_input() - input_id < 0");
                }
                if (input_id >= (int) n_inputs) {
                    n_inputs = input_id + 1;
                }
                inputs[input_id] = node_id;
            }

            void add_output(uint32_t node_id, int output_id) 
            {
                if (!is_neuron(node_id)) {
                    printf("risp::Network::add_output() - node %d "
                            "does not exist", (int)node_id);
                }

                else if (output_id < 0) {
                    printf("risp::Network::add_output() - output_id < 0");
                }

                else if (output_id >= (int) n_outputs) {
                    n_outputs++;
                }

                outputs[output_id] = node_id;
            }

            void process_events(uint32_t time) 
            {
                for (auto neuron : to_fire) {
                    neuron->perform_fire(time);
                }

                neuron_fire_counter += to_fire.size();
                to_fire.clear();

                const auto es = etl::move(events[time]);

                /* apply leak / reset minimum charge before the events happen */

                for (auto e : es) {
                    auto n = e.first;
                    if (n->leak) {
                        n->charge = 0;
                    }
                    if (n->charge < -1000.0) {
                        n->charge = -1000.0;
                    }
                }

                /* collect charges */

                for (auto e : es) {
                    auto n = e.first;
                    n->check = true;
                    n->charge += e.second;
                    neuron_accum_counter++;
                }

                auto events_size = events.size();

                /* determine if neuron fires */
                for (auto e : es) {

                    auto n = e.first;

                    if (n->check == true) {

                        /* fire */
                        if (n->charge >= n->threshold) {
                            for (size_t j=0; j<n->n_synapses; j++) {

                                auto syn = n->synapses[j];
                                const auto to_time = time + syn->delay;

                                if (to_time >= events_size) {
                                    events_size = to_time + 1;
                                    events.resize(events_size);
                                }

                                double weight = syn->weight;

                                if (0 != 0) {
                                    weight = rng.Random_Normal(weight,
                                            0);
                                }

                                events[to_time].push_back(
                                        etl::make_pair(syn->to, weight));

                            }

                            if (false) {
                                to_fire.push_back(n);
                            } else {
                                neuron_fire_counter++;
                                n->perform_fire(time);
                            }
                        }
                        n->check = false;
                    }
                }
            }

            void clear_tracking_info()
            {
                size_t i;
                Neuron *n;

                for (i = 0; i < sorted_neuron_vector.size(); i++) {
                    n = sorted_neuron_vector[i];
                    n->last_fire = -1;
                    n->fire_counts = 0;
                }
            }

            };

            class Processor : public neuro::Processor {

                private:

                    static bool is_integer(double v)
                    {
                        int iv = v;

                        return (iv == v);
                    }

                    Network * get_risp_network() 
                    {
                        return &((risp::Processor *)this)->risp_net;
                    }

                    risp::Network risp_net;

                public:

                    Processor() = default;

                    void apply_spike(const Spike& s, bool normalize)
                    {
                        get_risp_network()->apply_spike(s, normalize);
                    }

                    void run(double duration)
                    {
                        get_risp_network()->run(duration);
                    }

                    bool track_neuron_events(uint32_t node_id, bool track)
                    {
                        return get_risp_network()->track_neuron_events(
                                node_id, track);
                    }

                    etl::vector<int, 7> get_neuron_counts()
                    {
                        return get_risp_network()->get_neuron_counts();
                    }

                    etl::vector<double, 1> get_output_fire_times()
                    {
                        return get_risp_network()->get_output_fire_times();
                    }

                    etl::vector<double, 1> step(neuro::Network * net,
                            const double sim_time,
                            const float input1_spike_time,
                            const float input2_spike_time,
                            const float input3_spike_time)
                    {
                        neuro::apply_spike(net, this, 0, input1_spike_time);
                        neuro::apply_spike(net, this, 1, input2_spike_time);
                        neuro::apply_spike(net, this, 2, input3_spike_time);

                        run(sim_time);

                        return get_output_fire_times();
                    }

            };
    }
