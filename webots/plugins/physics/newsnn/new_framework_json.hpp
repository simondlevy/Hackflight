#pragma once

#include <fstream>

#include "new_framework.hpp"
#include "new_risp.hpp"

#include <nlohmann/json.hpp>

using namespace nlohmann;

namespace neuro
{
    class NetworkLoader {

        private:

            static void read_json(const char * filename, json &rv)
            {
                std::ifstream fin = {};
                rv.clear();
                fin.clear();
                fin.open(filename);
                fin >> rv; 
                fin.close();
            }

        public:

            static void load(
                    const json &j,
                    Network * net, 
                    risp::Processor & proc)
            {
                net->init();

                json jparams = j["Associated_Data"]["proc_params"];

                proc.load_network(net);

                EventTracker::track_all_neuron_events(&proc, net);
            }

            static void load(
                    const char * network_filename,
                    Network & net,
                    risp::Processor & proc)
            {
                json j = {};
                read_json(network_filename, j);
                NetworkLoader::load(j, &net, proc);
            }

    };
}   
