#include "new_risp.hpp"

void risp::Network::init_params()
{
    // XXX should come from compiler
    spike_value_factor=1000.000000;
    min_potential=-1000.000000;
    leak_mode=LEAK_NONE;
    run_time_inclusive=0;
    threshold_inclusive=1;
    fire_like_ravens=0;
    discrete=1;
    noisy_seed=0;
    noisy_stddev=0.000000;

}
