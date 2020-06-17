#ifndef CRDT_VERTEX
#define CRDT_VERTEX

#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <any>
#include <memory>
#include <vector>
#include <variant>
#include <qmat/QMatAll>
#include <typeinfo>
#include "topics/DSRGraphPubSubTypes.h"
#include <optional>
#include "dsr_exceptions.h"

namespace CRDT
{
    using N = Node;
    using Nodes = ormap<int, aworset<N,  int >, int>;
    using MTypes = std::variant<std::string, std::int32_t, float , std::vector<float>, bool, RMat::RTMat>;
    using IDType = std::int32_t;
    using AttribsMap = std::unordered_map<std::string, MTypes>;

    class VEdge
    {
        public:
            VEdge(Edge _edge) : edge(std::move(_edge)) {};
            VEdge(VEdge &vedge) : edge(std::move(vedge.edge)) {};
            VEdge() = delete; //{ edge.type("error"); };
            int to() const { return edge.to(); };
            int from() const { return edge.from(); };
            EdgeKey get_key() const { EdgeKey key; key.to(edge.to()); key.type(edge.type()); return key; };
            Edge& get_CRDT_edge() { return edge; }; // only for reinserting
            RMat::RTMat to_RT()
            {
                auto r = get_attrib_by_name<std::vector<float>>("rotation_euler_xyz");
                auto t = get_attrib_by_name<std::vector<float>>("translation");
                if( r.has_value() and t.has_value() )
                    return RTMat { r.value()[0], r.value()[1], r.value()[2], t.value()[0], t.value()[1], t.value()[2] } ;
                else return {};
            };
            std::optional<Attrib> get_attrib_by_name_(const std::string &key)
            {
                auto attrs = edge.attrs();
                auto value  = attrs.find(key);
                if (value != attrs.end())
                    return value->second;
                return {};
            }
            template <typename Ta>
            std::optional<Ta> get_attrib_by_name(const std::string &key)
            {
                std::optional<Attrib> av = get_attrib_by_name_(key);
                if (!av.has_value()) return {};
                if constexpr (std::is_same<Ta, std::string>::value) {
                    return av->value().str();
                }
                if constexpr (std::is_same<Ta, std::int32_t>::value){
                    return av->value().dec();
                }
                if constexpr (std::is_same<Ta, float>::value) {
                    return av->value().fl();
                }
                if constexpr (std::is_same<Ta, std::vector<float>>::value)
                {
                    return av->value().float_vec();
                }
                if constexpr (std::is_same<Ta, bool>::value)
                {
                    return av->value().bl();
                }
                if constexpr (std::is_same<Ta, RMat::QVec>::value)
                {
                    if (av.value().value()._d() == FLOAT_VEC and (key == "translation" or key=="rotation_euler_xyz")) 
                    {   
                        const auto &val = av.value().value().float_vec();
                        if (val.size()==3 or val.size()==6)
                            return QVec{val};
                        else return {};
                    }
                    else return {};
                }
                if constexpr (std::is_same<Ta, RMat::QMat>::value)
                {
                    if (av.value().value()._d() == FLOAT_VEC and key== "rotation_euler_xyz")
                    {
                        const auto& val = av.value().value().float_vec();
                        if(val.size() == 3)
                            return QMat{RMat::Rot3DOX(val[0])*RMat::Rot3DOY(val[1])*RMat::Rot3DOZ(val[2])};
                        else
                            return {};
                    }
                    else return {};
                }
                else return {};
            }
            void add_attrib(const std::string &key, int type, const Val &av)
            {
                Attrib at; at.type(type); at.value(av);
                edge.attrs().insert_or_assign(key, at);
            }
            void print()
            {
                std::cout << "------------------------------------" << std::endl;
                std::cout << "Edge-type->" << edge.type() << " from->" << edge.from() << " to->" << edge.to()  << std::endl;
                for(auto [k, v] : edge.attrs())
                    std::cout << "              Key->" << k << " Type->" << v.type() << " Value->" << v.value()  << std::endl;
                std::cout << "------------------------------------" << std::endl;
            }

        private:
            Edge edge;
    };

    using VEdgePtr = std::shared_ptr<VEdge>;

    class Vertex   
    {
        public:
            Vertex(N _node) : node(std::move(_node)) {};
            Vertex(Vertex &v) : node(std::move(v.node)) {};
            Vertex(Vertex &&v) : node(std::move(v.node)) {};
            // Faltan los constructores de copia
            Vertex() = delete; // forbid default constructor
            N& get_CRDT_node() { return node; };            // so it can be reinserted
            std::optional<std::int32_t> get_level()
            {
                return get_attrib_by_name<std::int32_t>("level");
            }
            std::string get_type() const { return node.type(); };
            std::optional<std::int32_t> get_parent()
            {
                return get_attrib_by_name<std::int32_t>("parent");
            }
            void add_attrib(std::map<string, Attrib> &v, std::string att_name, CRDT::MTypes att_value)
            {
                Attrib av;
                av.type(att_value.index());
                Val value;
                switch(att_value.index()) 
                {
                    case 0:
                        value.str(std::get<std::string>(att_value));
                        av.value( value);
                        break;
                    case 1:
                        value.dec(std::get<std::int32_t>(att_value));
                        av.value( value);
                        break;
                    case 2:
                        value.fl(std::get<float>(att_value));
                        av.value( value);
                        break;
                    case 3:
                        value.float_vec(std::get<std::vector<float>>(att_value));
                        av.value( value);
                        break;
                    case 4:
                        value.bl(std::get<bool>(att_value));
                        av.value( value);
                        break;
                }
                v[att_name] = av;
            }
            std::int32_t id() const { return node.id(); };
            std::string type() const { return node.type(); };
            std::optional<Attrib> get_attrib_by_name_(const std::string &key)
            {
                auto attrs = node.attrs();
                auto value  = attrs.find(key);
                if (value != attrs.end())
                    return value->second;
                return {};
            }
            template <typename Ta>
            std::optional<Ta> get_attrib_by_name(const std::string &key)
            {
                std::optional<Attrib> av = get_attrib_by_name_(key);
                if (!av.has_value()) return {};
                if constexpr (std::is_same<Ta, std::string>::value) {
                    return av->value().str();
                }
                if constexpr (std::is_same<Ta, std::int32_t>::value){
                    return av->value().dec();
                }
                if constexpr (std::is_same<Ta, float>::value) {
                    return av->value().fl();
                }
                if constexpr (std::is_same<Ta, std::vector<float>>::value)
                {
                    return av->value().float_vec();
                }
                if constexpr (std::is_same<Ta, bool>::value)
                {
                    return av->value().bl();
                }
                throw(DSRException("Illegal Return type"));
            }

            // Edges
            std::optional<VEdgePtr> get_edge(int to, const std::string& key)
            {
                EdgeKey ek;
                ek.to(to);
                ek.type(key);
                auto edge = node.fano().find(ek);
                if (edge != node.fano().end())
                    return std::make_shared<VEdge>(Edge(edge->second));
                return {};
            }
            void insert_or_assign_edge(const VEdgePtr& vedge)
            {
                node.fano().insert_or_assign(vedge->get_key(), vedge->get_CRDT_edge());
            }
            std::vector<VEdgePtr> get_edges()
            {
                std::vector<VEdgePtr> res;
                for(auto &[k,v]: node.fano())
                    res.emplace_back(std::make_shared<VEdge>(v));
                return res;
            }
            std::vector<VEdgePtr> get_edges_by_type(const std::string& type);
            std::vector<VEdgePtr> get_edges_to_id(int id);
            std::optional<RTMat> get_edge_RT(int to) 
            {
                auto ve = get_edge(to, "RT");
                if(ve.has_value())
                    return ve.value()->to_RT();
                else
                    return {};               
            }
            //add_edge(const VEdge& v) 

            // Utils
            void print()
            {   
                std::cout << "------------------------------------" << std::endl;
                std::cout << "Node-> " << node.id() << std::endl;
                std::cout << "  Type->" << node.type() << std::endl;
                std::cout << "  Name->" << node.name() << std::endl;
                std::cout << "  Agent_id->" << node.agent_id()  << std::endl;
                for(auto [key, val] : node.attrs())
                std::cout << "      Key->" << key << " Type->" << val.type() << " Value->" << val.value()  << std::endl;
                for(auto [key, val] : node.fano())
                {
                    std::cout << "          Edge-type->" << val.type() << " from->" << val.from() << " to->" << val.to()  << std::endl;
                    for(auto [k, v] : val.attrs())
                    std::cout << "              Key->" << k << " Type->" << v.type() << " Value->" << v.value()  << std::endl;
                }
                std::cout << "------------------------------------" << std::endl;
            }

        private:
            N node;
    };




} // namespace CRDT

#endif
