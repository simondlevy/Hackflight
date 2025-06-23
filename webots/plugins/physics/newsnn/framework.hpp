// AUTO-GENERATED; DO NOT EDIT

#pragma once

#include <Embedded_Template_Library.h>
#include <etl/vector.h>

namespace neuro
{
    typedef etl::pair<int,int> Coords;

    class Node {

        private:

            // arbitrary limits
            static const size_t MAX_INCOMING = 20;
            static const size_t MAX_OUTGOING = 20;

        public:

            Node() = default;

            uint32_t id = 0;                
            int input_id = -1;              
            int output_id = -1;             
            etl::vector<class Edge*, MAX_INCOMING> incoming;         
            etl::vector<class Edge*, MAX_OUTGOING> outgoing;         

            double threshold;

            inline bool is_input() const                
            { return input_id >= 0; }

            inline bool is_output() const               
            { return output_id >= 0; }
    };

    class Edge {

        public:

            Edge() = default;

            Node* from;                         
            Node* to;                           

            double weight;
            double delay;
    };

    struct Spike
    {
        int id; 
        double time; 
        double value; 

        Spike(int id_, double time_, double value_)
            : id(id_), time(time_), value(value_) {}
    };

    class Processor {

        public:

            virtual void apply_spike(
                    const Spike& s, bool normalized = true) = 0;

            virtual bool track_neuron_events(
                    uint32_t node_id, bool track = true) = 0;

            virtual void run(double duration) = 0;

            virtual etl::vector <int, 7> get_neuron_counts() = 0;
    };

    class Network {

        friend class NetworkLoader;

        public:

            Network() = default;

            void clear()
            {
                inputs.clear();
                outputs.clear();
                sorted_node_vector.clear();

                n_edges = 0;
                n_nodes = 0;
            }

            Node* add_node(const uint32_t idx, const double threshold)
            {
                Node * node = nullptr;

                if (n_nodes+1 == 7) {
                    printf("Network::add_node: would excede maximum of 7");
                }

                else {

                    node = &nodes[n_nodes];

                    node->id = idx;
                    node->threshold = threshold;

                    n_nodes++;
                }

                return node;
            }

            Edge* add_edge(uint32_t fr, uint32_t to,
                    double weight, double delay)
            {

                Edge * edge = nullptr;

                if (is_edge(fr, to)) {
                    printf("Edge %d -> %d already exists.", (int)fr, (int)to);
                }

                else if (n_edges+1 == 13) {
                    printf("Network::add_edge: would excede maximum of 13\n");
                }

                else {

                    Node * from_node = get_node(fr);
                    Node * to_node = get_node(to);

                    edge = &edges[n_edges];

                    edge->from = from_node;
                    edge->to = to_node;
                    edge->weight = weight;
                    edge->delay = delay;

                    from_node->outgoing.push_back(edge);
                    to_node->incoming.push_back(edge);

                    n_edges++;
                }

                return edge;
            }

            bool is_edge(uint32_t fr, uint32_t to)
            {
                return get_edge(fr, to) != nullptr;
            }

            Edge * get_edge(uint32_t fr, uint32_t to) 
            {
                Edge * edge = nullptr;

                for (size_t k=0; k<n_edges; ++k) {
                    Edge * e = &edges[k];
                    if (e->from->id == fr && e->to->id == to) {
                        edge = e;
                    }
                }

                return edge;
            }

            Node* get_node(uint32_t idx)
            {
                Node * node = nullptr;

                for (size_t k=0; k<n_nodes; ++k) {
                    if (idx == nodes[k].id) {
                        node = &nodes[k];
                    }
                }

                return node;
            }

            int add_input(uint32_t idx)
            {
                Node *n = get_node(idx);

                if (n->input_id >= 0) {
                    printf("Node %d was already set as an input (%d).",
                            (int)idx, (int)n->input_id);
                }

                n->input_id = inputs.size();
                inputs.push_back(idx);
                return n->input_id;
            }

            int add_output(uint32_t idx)
            {
                Node *n = get_node(idx);
                if (n->output_id >= 0) {
                    printf("Node %d was already set as an output (%d).\n",
                            (int)idx, (int)n->output_id);
                }
                n->output_id = outputs.size();
                outputs.push_back(idx);
                return n->output_id;
            }

            int num_outputs() const
            {
                return outputs.size();
            }

            static bool node_comp(Node *n1, Node *n2)
            {
                return (n1->id < n2->id); 
            }

            void make_sorted_node_vector()
            {
                if (sorted_node_vector.size() == 0) {

                    for (size_t k=0; k<n_nodes; ++k) {
                        sorted_node_vector.push_back(&nodes[k]);
                    }

                    etl::sort(sorted_node_vector.begin(),
                            sorted_node_vector.end(), node_comp); 
                }
            }

            etl::vector <Node *, 7> sorted_node_vector;   

            Node nodes[7];
            size_t n_nodes;

            Edge edges[13];
            size_t n_edges;

        protected:

            etl::vector<uint32_t, 3> inputs;
            etl::vector<uint32_t, 1> outputs;

            friend class Node;
            friend class Edge;
    };

    class EventTracker {

        public:

            static void track_all_neuron_events(Processor * proc,
                    Network * net)
            {
                for (size_t k=0; k<net->n_nodes; ++k) {
                    proc->track_neuron_events(net->nodes[k].id, true);
                }
            }
    };

    static void apply_spike(
            Network * net,
            Processor *p,
            const int spike_id,
            const double spike_time,
            const double spike_val=1,
            const bool normalize=true) 
    {
        p->apply_spike(Spike(net->get_node(spike_id)->input_id,
                    spike_time, spike_val), normalize);

    }
}

