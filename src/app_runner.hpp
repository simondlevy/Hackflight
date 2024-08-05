/*
   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.

 */

#pragma once

#include <vector>
#include <string>

#include <framework.hpp>
#include <io_stream.hpp>
#include <jspace.hpp>
#include <nlohmann/json.hpp>
#include <utils/json_helpers.hpp>

using std::string;
using std::vector;
using std::ifstream;

typedef std::runtime_error SRE;

class App_Runner
{
    public:

        App_Runner(
                const string network_filename, const string proc_name)
        {
            ifstream fin;
            nlohmann::json j;

            fin.open(network_filename.c_str());
            if (fin.fail()) {
                throw SRE((string) "Couldn't open network file: " + 
                        network_filename);
            }
            fin >> j;

            neuro::Network template_network;    

            template_network.from_json(j);

            auto proc_params = template_network.get_data("proc_params");

            proc = neuro::Processor::make(proc_name, proc_params);

            net = new neuro::Network(template_network);

            encoder_array.from_json(net->get_data("encoder_array"));

            decoder_array.from_json(net->get_data("decoder_array"));

            auto other = net->get_data("other");

            sim_time = other["sim_time"];

            if (!(proc->load_network(net, 0))) {
                throw SRE("Load network failed");
            }
        }

        ~App_Runner()
        {
            delete proc;

            delete net;
        }

        void getActions(vector <double>  &o, vector <double> &a)
        {
            proc->clear_activity();

            auto spikes = encoder_array.get_spikes(o);

            proc->apply_spikes(spikes, 0);

            proc->run(sim_time, 0);

            vector <double> times;
            vector <int> counts;
            decoder_array.get_output_counts_and_times(counts, times, *(proc), 0);

            a = decoder_array.get_data(counts, times);
        }

    private:

        neuro::Network * net;

        neuro::Processor * proc;

        neuro::EncoderArray encoder_array;

        neuro::DecoderArray decoder_array;

        int sim_time;

        // This is unused, but leaving it out will cause link errors
        neuro::IO_Stream prompt_stream;    
};
