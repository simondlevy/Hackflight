#pragma once

#include <fstream>

#include "new_framework.hpp"
#include "new_risp.hpp"

#include <nlohmann/json.hpp>

using namespace nlohmann;

namespace neuro
{
    class NetworkLoader {

        public:

            static void load(Network * net, risp::Processor & proc)
            {
                net->init();

                proc.load_network(net);

                EventTracker::track_all_neuron_events(&proc, net);
            }
    };
}   
