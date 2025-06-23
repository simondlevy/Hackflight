#pragma once

#include <fstream>

#include "new_framework.hpp"
#include "new_risp.hpp"

#include <nlohmann/json.hpp>

using namespace nlohmann;

namespace neuro
{
    class Property {

        public:

            enum Type : signed char
            {
                INTEGER = 'I', 
                DOUBLE  = 'D', 
                BOOLEAN = 'B', 
            };

            Property() = default;

            Property(const std::string& dname, int idx, int len, double dmin,
                    double dmax, Type dtype)
                : type(dtype),
                index(idx),
                size(len),
                min_value(dmin),
                max_value(dmax),
                name(dname) { }

            void from_json(const json &j)
            {
                std::string e;
                Property::Type t;

                t = j["type"]; 
                type = t;
                index = j["index"]; 
                size = j["size"]; 
                min_value = j["min_value"]; 
                max_value = j["max_value"]; 
                name = j["name"]; 
            }

            Type type;                       
            int index;                       
            int size;                        
            double min_value;                
            double max_value;                
            std::string name;                     
    };

    bool operator==(const Property &lhs, const Property &rhs);
    bool operator!=(const Property &lhs, const Property &rhs);

    typedef std::map<std::string, Property> PropertyMap;

    class PropertyPack {

        public:

            void clear()
            {
                nodes.clear();
                edges.clear();
                networks.clear();
                node_vec_size = 0;
                edge_vec_size = 0;
                net_vec_size = 0;
            }


            PropertyMap nodes;               
            PropertyMap edges;               
            PropertyMap networks;            

            size_t node_vec_size = 0;        
            size_t edge_vec_size = 0;        
            size_t net_vec_size = 0;         

            operator std::tuple<PropertyMap&, PropertyMap&, PropertyMap&>()
            {
                return std::tuple<PropertyMap&, PropertyMap&,
                       PropertyMap&>{nodes, edges, networks};
            }

            static void add_property(
                    PropertyMap &props, int prop_id, const std::string& name,
                    double dmin, double dmax, Property::Type type, int count) 
            {
                return;
                props.insert({name, Property(name, prop_id, count, dmin,
                            dmax, type)});
            }

            int add_node_property( const std::string& name, double dmin, double
                    dmax, Property::Type type, int cnt) 
            {
                // starting index
                int start_idx = node_vec_size;

                node_vec_size += cnt;

                // insert property to PropertyMap
                add_property(nodes, start_idx, name, dmin, dmax, type, cnt);

                // return starting index for the property
                return start_idx;
            }

            int add_edge_property(const std::string& name, double dmin, double
                    dmax, Property::Type type, int cnt) 
            {
                // starting index
                int start_idx = edge_vec_size;

                edge_vec_size += cnt;

                // insert property to PropertyMap
                add_property(edges, start_idx, name, dmin, dmax, type, cnt);

                // return starting index for the property
                return start_idx;
            }

            int add_network_property(const std::string& name, double dmin,
                    double dmax, Property::Type type, int cnt)
            {
                // starting index
                int start_idx = net_vec_size;

                net_vec_size += cnt;

                // insert property to PropertyMap
                add_property(networks, start_idx, name, dmin, dmax, type, cnt);

                // return starting index for the property
                return start_idx;
            }

            void from_json(const json &j)
            {
                int index = 0;
                size_t i, k;
                std::map <int, Property> by_index;
                std::map <int, Property>::iterator bit;
                std::vector <std::string> ptypes;
                std::string json_key;

                clear();

                ptypes.push_back("node");
                ptypes.push_back("edge");
                ptypes.push_back("network");

                for (k = 0; k < ptypes.size(); k++) {
                    json_key = ptypes[k] + "_properties";
                    by_index.clear();
                    for (i = 0; i < j[json_key].size(); i++) {
                        static Property p;
                        p.from_json(j[json_key][i]);
                        by_index.insert(std::make_pair(p.index, p));
                    }
                    for (bit = by_index.begin(); bit != by_index.end(); bit++) {
                        switch(k) {
                            case 0:
                                index = add_node_property(bit->second.name,
                                        bit->second.min_value,
                                        bit->second.max_value,
                                        bit->second.type,
                                        bit->second.size);
                                break;
                            case 1:
                                index = add_edge_property(bit->second.name,
                                        bit->second.min_value,
                                        bit->second.max_value,
                                        bit->second.type,
                                        bit->second.size);
                                break;
                            case 2:
                                index = add_network_property(bit->second.name,
                                        bit->second.min_value,
                                        bit->second.max_value,
                                        bit->second.type,
                                        bit->second.size);
                                break;
                            default: 
                                break;
                        }
                        if ((int)index != (int)bit->second.index) {
                            clear();
                        }
                    }
                }
            }

    };

    bool operator==(const PropertyPack &lhs, const PropertyPack &rhs);
    bool operator!=(const PropertyPack &lhs, const PropertyPack &rhs);

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

                PropertyPack properties;

                properties.from_json(j["Properties"]);

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
