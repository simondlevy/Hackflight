/*
   Spiking Neural Net support using TeNNLab framework

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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

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

class SNN
{
    public:

        SNN(const string network_filename, const string proc_name)
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

        ~SNN()
        {
            delete proc;

            delete net;
        }

        void step(vector <double>  &observations, vector <double> &actions)
        {
            proc->clear_activity();

            auto spikes = encoder_array.get_spikes(observations);

            proc->apply_spikes(spikes, 0);

            proc->run(sim_time, 0);

            vector <int> counts;
            vector <double> times;
            decoder_array.get_output_counts_and_times(counts, times, *(proc), 0);

            actions = decoder_array.get_data(counts, times);
        }

        void serve_visualizer(const int port)
        {
            // Serve up a socket for the visualizer
            printf("Listening for viz client on port %d ...", port);
            fflush(stdout);

            const auto listenfd = socket(AF_INET, SOCK_STREAM, 0);

            struct sockaddr_in serv_addr = {};

            serv_addr.sin_family = AF_INET;

            serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

            serv_addr.sin_port = htons(port);

            bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

            listen(listenfd, 1);

            viz_client = accept(listenfd, (struct sockaddr*)NULL, NULL);

            printf("\nClient connected\n");
        }

        bool send_counts_to_visualizer(void)
        {
            const auto counts = proc->neuron_counts();

            string msg = "{\"Event Counts\":[";

            char tmp[100];

            const auto n1 = counts.size() - 1;

            for (size_t i=0; i<n1; ++i) {
                sprintf(tmp, "%d,", counts[i]);
                msg += tmp;
            }

            sprintf(tmp, "%d],\"Neuron Alias\":[", counts[n1]);
            msg += tmp;

            for (size_t i=0; i<n1; ++i) {
                sprintf(tmp, "%d,", net->sorted_node_vector[i]->id);
                msg += tmp;
            }

            sprintf(tmp, "%d]}\n", net->sorted_node_vector[n1]->id);

            msg += tmp;

            const auto len = msg.size();

            return write(viz_client, msg.c_str(), len) == (int)len;
        }

    private:

        neuro::Network * net;

        neuro::Processor * proc;

        neuro::EncoderArray encoder_array;

        neuro::DecoderArray decoder_array;

        int sim_time;

        int viz_client;

        // This is unused, but leaving it out will cause link errors
        neuro::IO_Stream prompt_stream;    
};


