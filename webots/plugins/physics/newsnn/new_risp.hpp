#pragma once

#include <etl/unordered_map.h>

#include "new_framework.hpp"
#include <utils/MOA.hpp>    

using namespace neuro;

namespace risp
{

    // Arbitrary array limits
    static const size_t MAX_EVENT_TIMES = 4000;
    static const size_t MAX_EVENTS_PER_TIME = 10;

    typedef enum {

        LEAK_NONE,
        LEAK_ALL,
        LEAK_CONFIGURABLE
    } leak_mode_t;

    class Params {

        public:

            double spike_value_factor; 
            double min_potential;
            leak_mode_t leak_mode;
            bool run_time_inclusive;
            bool threshold_inclusive;
            bool fire_like_ravens;
            bool discrete;
            uint32_t noisy_seed;
            double noisy_stddev;
    };

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
                // XXX should come from compiler
                params.spike_value_factor=1000.000000;
                params.min_potential=-1000.000000;
                params.leak_mode=LEAK_NONE;
                params.run_time_inclusive=0;
                params.threshold_inclusive=1;
                params.fire_like_ravens=0;
                params.discrete=1;
                params.noisy_seed=0;
                params.noisy_stddev=0.000000;

                n_neurons = 0;
                n_synapses = 0;
                n_inputs = 0;
                overall_run_time = 0;
                neuron_fire_counter = 0;
                neuron_accum_counter = 0;

                rng.Seed(params.noisy_seed, "noisy_risp");

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
                    v = params.discrete ?
                        floor(s.value * params.spike_value_factor) :
                        s.value * params.spike_value_factor;
                } else {
                    v = s.value;
                }

                if (params.noisy_stddev != 0) {
                    v = rng.Random_Normal(v, params.noisy_stddev);
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

                int run_time =
                    (params.run_time_inclusive) ?  duration : duration-1;

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
                    if (neuron->charge < params.min_potential) {
                        neuron->charge = params.min_potential;
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

            etl::vector<double, MAX_NODES> get_output_fire_times()
            {
                etl::vector<double, MAX_NODES> times;

                for (size_t k=0; k<n_outputs; ++k) {
                   times.push_back(neuron_map[outputs[k]]->last_fire);
                }

                return times;
            }

            etl::vector<int, MAX_NODES> get_neuron_counts() 
            {
                etl::vector<int, MAX_NODES> counts;

                for (auto neuron : sorted_neuron_vector) {
                    counts.push_back(neuron->fire_counts);
                }

                return counts;
            }

                protected:

            Params params;

            size_t n_neurons;
            Neuron neurons[MAX_NODES];

            size_t n_synapses;
            Synapse synapses[MAX_SYNAPSES];

            int inputs[MAX_INPUTS];
            size_t n_inputs;

            int outputs[MAX_OUTPUTS];
            size_t n_outputs;

            etl::vector<Neuron *, MAX_NODES> sorted_neuron_vector;         

            etl::unordered_map <uint32_t, Neuron *, MAX_NODES> neuron_map;   

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
            etl::vector<Neuron *, MAX_NODES> to_fire;   

            Neuron * add_neuron(uint32_t node_id, double threshold, bool leak) 
            {
                Neuron * n = nullptr;

                if (is_neuron(node_id)) {
                    printf("risp::Neuron::add_neuron() - %d is already "
                            "in the neuron map\n", (int)node_id);
                }

                if (n_neurons+1 == MAX_NODES) {
                    printf("Network::add_neuron: max of %d neurons exceeded\n", 
                            (int)MAX_NODES);
                }

                else {

                    neurons[n_neurons].init(node_id, threshold, leak);

                    n = &neurons[n_neurons];

                    n_neurons++;

                    /* JSP: I'm not a big fan of this hack, 
                       but I'd rather do this than put an if
                       statement before every threshold check.  */
                    if (!params.threshold_inclusive) {
                        n->threshold = (params.discrete) ?
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
                    if (n->charge < params.min_potential) {
                        n->charge = params.min_potential;
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

                                if (params.noisy_stddev != 0) {
                                    weight = rng.Random_Normal(
                                            weight, params.noisy_stddev);
                                }

                                events[to_time].push_back(
                                        etl::make_pair(syn->to, weight));

                            }

                            if (params.fire_like_ravens) {
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

                public:

                    Processor() = default;

                    void load_network(neuro::Network* net) 
                    {
                        risp_net.init(net);
                    }

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

                    etl::vector<int, MAX_NODES> get_neuron_counts()
                    {
                        return get_risp_network()->get_neuron_counts();
                    }

                    etl::vector<double, MAX_NODES> get_output_fire_times()
                    {
                        return get_risp_network()->get_output_fire_times();
                    }

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

                    risp::Params params;
            };

    }
