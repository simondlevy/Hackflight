#include "new_framework.hpp"

void neuro::Network::init_params()
{
    // XXX should come from compiler
    add_node(0, 1.000000);
    add_node(1, 1.000000);
    add_node(2, 1.000000);
    add_node(3, 1.000000);
    add_node(4, 1.000000);
    add_node(5, 1000.000000);
    add_node(6, 1.000000);
    add_edge(0, 3, 1.000000, 1.000000);
    add_edge(0, 4, -1.000000, 1.000000);
    add_edge(1, 3, -1.000000, 1.000000);
    add_edge(1, 4, 1.000000, 1.000000);
    add_edge(2, 6, 1.000000, 1000.000000);
    add_edge(3, 3, 1.000000, 1.000000);
    add_edge(3, 5, -1.000000, 1.000000);
    add_edge(4, 4, 1.000000, 1.000000);
    add_edge(4, 5, 1.000000, 1.000000);
    add_edge(5, 5, -1.000000, 1.000000);
    add_edge(5, 6, -1.000000, 1.000000);
    add_edge(6, 5, 1.000000, 1.000000);
    add_edge(6, 6, 1.000000, 1.000000);
    add_input(0);
    add_input(1);
    add_input(2);
    add_output(5);
}
