#pragma once

#include <fstream>
#include <sstream>

#include "new_framework.hpp"
#include "new_risp.hpp"

#include <nlohmann/json.hpp>

using namespace std;
using namespace nlohmann;

typedef runtime_error SRE;

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

            Property(const json& j)
            {
                from_json(j);
            }

            Property(const Property& p) :
                type(p.type), 
                index(p.index),
                size(p.size),
                min_value(p.min_value),
                max_value(p.max_value),
                name(p.name) { }

            Property(Property &&p) noexcept :
                type(p.type), 
                index(p.index),
                size(p.size),
                min_value(p.min_value), 
                max_value(p.max_value),
                name(std::move(p.name)) { }

            json as_json() const
            {
                json j = json::object();
                to_json(j);
                return j;
            }

            void from_json(const json &j)
            {
                std::string e;
                Property::Type t;

                t = j["type"]; 
                if (t != Property::Type::BOOLEAN && 
                        t != Property::Type::INTEGER && 
                        t != Property::Type::DOUBLE) {
                    printf("Bad Property JSON - Type must be D (%d), I (%d) "
                            "or B (%d).\n", 'D', 'I', 'B');
                }
                type = t;
                index = j["index"]; 
                size = j["size"]; 
                min_value = j["min_value"]; 
                max_value = j["max_value"]; 
                name = j["name"]; 
            }

            void to_json(json &j) const
            {
                j["type"] = type;
                j["index"] = index;
                j["size"] = size;
                j["min_value"] = min_value;
                j["max_value"] = max_value;
                j["name"] = name;
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

            void to_json(json &j) const;     

            json as_json() const;            

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
                if(count <= 0) printf("Property count must be > 0.");
                if(name.empty()) printf("Property name must not be empty.");
                if(props.find(name) != props.end()) {
                    printf("Property name %s already exists.\n", name.c_str());
                }

                props.insert({name, Property(name, prop_id, count, dmin,
                            dmax, type)});
            }

            int add_node_property( const std::string& name, double dmin, double
                    dmax, Property::Type type, int cnt) 
            {
                // starting index
                int start_idx = node_vec_size;
                if(cnt < 1) printf("Count must be > 0.\n");

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
                if(cnt < 1) printf("Count must be > 0.\n");

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
                if(cnt < 1) printf("Count must be > 0.\n");

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
                                printf("Switch in PropertyPack::from_json\n");
                        }
                        if ((int)index != (int)bit->second.index) {
                            clear();
                            printf("Property Pack: non-matching index in %s "
                                    "json: %s\n", ptypes[k].c_str(),
                                    bit->second.as_json().dump().c_str()); 
                        }
                    }
                }
            }

    };

    bool operator==(const PropertyPack &lhs, const PropertyPack &rhs);
    bool operator!=(const PropertyPack &lhs, const PropertyPack &rhs);

    class NetworkLoader {

        private:

            static bool read_json(const char * filename, json &rv)
            {
                bool success = true;

                string s = {};

                ifstream fin = {};

                rv.clear();

                fin.clear();

                fin.open(filename);

                try {
                    fin >> rv; success = true;
                } catch(...) {
                    success = false;
                }

                fin.close();

                return success;
            }
        public:

            static void load(const json &j, Network * net, 
                    risp::Processor & proc)
            {
                net->clear();

                PropertyPack properties;

                properties.from_json(j["Properties"]);

                // Add the Network values

                const auto values =
                    j["Network_Values"].get<std::vector <double>>();
                if (values.size() != properties.net_vec_size) {
                    printf("Error in network JSON: "
                            "Network_Value's array's size doesn't match the "
                            "network Propery Pack\n");
                    return;
                }

                // Add nodes /w values CHZ I didn't pass as reference because
                // we may need to modify jn

                for(auto jn : j["Nodes"]) {   

                    const auto jnid = jn["id"];

                    const auto values = jn["values"].get<std::vector<double>>();

                    const double threshold = values[0];

                    printf("Add node: %s %f\n",
                            jnid.dump().c_str(), threshold);
                    net->add_node(jnid, threshold);

                    const auto nsize = values.size();

                    const auto psize = properties.node_vec_size;
                    if (nsize != psize) {
                        printf("Error in network JSON: Node %s array size %d "
                                "does not match node PropertyPack size %d\n",
                                jnid.dump().c_str(), (int)nsize, (int)psize);
                        return;
                    }
                }

                // Add edges /w values
                for(auto& je : j["Edges"]) {

                    const auto values = je["values"].get<std::vector<double>>();

                    net->add_edge(je["from"], je["to"], values[0], values[1]);

                    printf("add_edge(%s, %s, %f %f);\n", 
                            je["from"].dump().c_str(),
                            je["to"].dump().c_str(),
                            values[0],
                            values[1]);

                    if (values.size() != properties.edge_vec_size) {
                        std::string estring =
                            "Error in the network JSON: Edge " +
                            je["from"].dump() +
                            "->" + je["to"].dump() + 
                            "'s value array's size does not match the edge "
                            "PropertyPack";
                        printf("%s\n", estring.c_str());
                        return;
                    }
                }

                // Add the inputs & outputs

                for (size_t i = 0; i < j["Inputs"].size(); i++) {
                    if (j["Inputs"][i].get<double>() < 0) {
                        char buf[128];
                        snprintf(buf, 128, "%d", (int) i);
                        std::string estring =
                            (std::string) "Bad Network JSON - Input[" +
                            (std::string) buf + "] is < 0.";
                        printf("%s\n", estring.c_str());
                        return;
                    }
                    printf("add_input(%u\n", j["Inputs"][i].get<uint32_t>());
                    net->add_input(j["Inputs"][i].get<uint32_t>());
                }

                for (size_t i = 0; i < j["Outputs"].size(); i++) {
                    if (j["Outputs"][i].get<double>() < 0) {
                        char buf[128];
                        snprintf(buf, 128, "%d", (int) i);
                        std::string estring =
                            (std::string) "Bad Network JSON - Output[" +
                            (std::string) buf + "] is < 0.";
                        printf("%s ", estring.c_str());
                        return;
                    }
                    net->add_output(j["Outputs"][i].get<uint32_t>());
                    printf("add_output(%u\n", j["Outputs"][i].get<uint32_t>());
                }

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

                if (!read_json(network_filename, j)) {

                    printf("usage: ML j. Bad json\n");

                } else {

                    try {

                        NetworkLoader::load(j, &net, proc);

                    } catch (const SRE &e) {
                        printf("%s\n",e.what());
                    } catch (...) {
                        printf("Unknown error when making processor\n");
                    }
                }
            }

    };
}   
