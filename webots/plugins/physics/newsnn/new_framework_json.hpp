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

                risp::Params params = {};

                params.min_potential = jparams["min_potential"];

                params.discrete = jparams["discrete"];

                if (jparams.contains("threshold_inclusive")) {
                    params.threshold_inclusive = jparams["threshold_inclusive"];
                }

                if (jparams.contains("spike_value_factor")) {
                    params.spike_value_factor = jparams["spike_value_factor"];
                } 

                else {
                    params.spike_value_factor = 0; // max_weight;

                } 

                if (jparams.contains("run_time_inclusive")) {
                    params.run_time_inclusive = jparams["run_time_inclusive"];
                }

                if (jparams.contains("fire_like_ravens")) {
                    params.fire_like_ravens = jparams["fire_like_ravens"];
                }

                if (jparams.contains("noisy_seed")) {
                    params.noisy_seed = jparams["noisy_seed"];
                }

                if (jparams.contains("leak_mode")) {
                    const auto mode_string = jparams["leak_mode"];
                    if (mode_string == "all") {
                        params.leak_mode = risp::LEAK_ALL;
                    }
                    if (mode_string == "configurable") {
                        params.leak_mode = risp::LEAK_CONFIGURABLE;
                    }
                }

                if (jparams.contains("noisy_stddev")) {
                    params.noisy_stddev = jparams["noisy_stddev"]; 
                }

                proc.init(params);

                if (!proc.load_network(net)) {
                    printf("loadnetwork() failed");
                }

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
