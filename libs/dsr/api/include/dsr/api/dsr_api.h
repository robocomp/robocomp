//
// Created by crivac on 17/01/19.
//

#ifndef DSR_GRAPH
#define DSR_GRAPH

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
#include <optional>
#include <type_traits>
#include "dsr/api/dsr_eigen_defs.h"
#include "dsr/core/crdt/delta_crdt.h"
#include "dsr/core/rtps/dsrparticipant.h"
#include "dsr/core/rtps/dsrpublisher.h"
#include "dsr/core/rtps/dsrsubscriber.h"
#include "dsr/core/topics/IDLGraphPubSubTypes.h"
#include "dsr/core/types/crdt_types.h"
#include "dsr/core/types/user_types.h"
#include "dsr/core/types/translator.h"
#include "dsr/api/dsr_inner_api.h"
#include "dsr/api/dsr_agent_info_api.h"
#include "dsr/api/dsr_inner_eigen_api.h"
#include "dsr/api/dsr_camera_api.h"
#include "dsr/api/dsr_rt_api.h"
#include "dsr/api/dsr_utils.h"
#include "dsr/core/types/type_checking/dsr_attr_name.h"
#include "dsr/core/types/type_checking/dsr_node_type.h"
#include "dsr/core/types/type_checking/dsr_edge_type.h"
#include "dsr/core/utils.h"
#include "dsr/core/id_generator.h"
#include "threadpool/threadpool.h"


#define TIMEOUT 5000

namespace DSR
{
    using Nodes = std::unordered_map<uint64_t , mvreg<CRDTNode>>;
    using IDType = uint64_t;

    /////////////////////////////////////////////////////////////////
    /// CRDT API
    /////////////////////////////////////////////////////////////////
    class DSRGraph : public QObject
    {
        friend RT_API;
        Q_OBJECT

        public:
        size_t size();
        DSRGraph(uint64_t root, std::string name, int id, const std::string& dsr_input_file = std::string(), bool all_same_host = true);
        ~DSRGraph() override;


        //////////////////////////////////////////////////////
        ///  Graph API
        //////////////////////////////////////////////////////

        // Utils
        bool empty(const uint64_t &id);
        std::map<uint64_t, Node> getCopy() const;

        [[deprecated("Use get_inner_eigen_api instead")]] std::unique_ptr<InnerAPI> get_inner_api() { return std::make_unique<InnerAPI>(this); };
        std::unique_ptr<InnerEigenAPI> get_inner_eigen_api() { return std::make_unique<InnerEigenAPI>(this); };
        std::unique_ptr<RT_API> get_rt_api() { return std::make_unique<RT_API>(this); };
        std::unique_ptr<CameraAPI> get_camera_api(const DSR::Node &camera_node) { return std::make_unique<CameraAPI>(this, camera_node); };


        //////////////////////////////////////////////////////
        ///  Core API
        //////////////////////////////////////////////////////

        // Nodes
        std::optional<Node> get_node(const std::string &name);
        std::optional<Node> get_node(uint64_t id);
        std::optional<uint64_t> insert_node(Node &node);
        bool update_node(Node &node);
        bool delete_node(const std::string &name);
        bool delete_node(uint64_t id);

        // Edges
        bool insert_or_assign_edge(const Edge& attrs);
        std::optional<Edge> get_edge(const std::string& from, const std::string& to, const std::string& key);
        std::optional<Edge> get_edge(uint64_t from, uint64_t to, const std::string& key);
        std::optional<Edge> get_edge(const Node& n, const std::string& to, const std::string& key);
        static std::optional<Edge> get_edge(const Node& n, uint64_t to, const std::string& key);
        bool delete_edge(const std::string& from, const std::string& t, const std::string& key);
        bool delete_edge(uint64_t from, uint64_t t, const std::string& key);
        /**CORE END**/


        //////////////////////////////////////////////////////
        ///  CONVENIENCE METHODS
        //////////////////////////////////////////////////////
        // Nodes
        std::optional<Node> get_node_root() { return get_node("world"); };
        std::vector<Node> get_nodes_by_type(const std::string &type);
        std::vector<Node> get_nodes_by_types(const std::vector<std::string> &types);
        std::optional<std::string> get_name_from_id(uint64_t id);
        std::optional<uint64_t> get_id_from_name(const std::string &name);
        std::optional<std::int32_t> get_node_level(const Node &n);
        std::optional<uint64_t> get_parent_id(const Node &n);
        std::optional<Node> get_parent_node(const Node &n);
        [[deprecated("Use node.type() instead")]]static std::string get_node_type(Node &n);

        // Edges
        std::vector<Edge> get_edges_by_type(const std::string &type);
        static std::vector<Edge> get_node_edges_by_type(const Node &node, const std::string &type);
        std::vector<Edge> get_edges_to_id(uint64_t id);
        std::optional<std::map<std::pair<uint64_t, std::string>, Edge>> get_edges(uint64_t id);


        template <typename name, typename Type>
        std::optional<uint64_t> get_attrib_timestamp(const Type &n)
            requires(node_or_edge<Type> and is_attr_name<name>)
        {
            auto &attrs = n.attrs();
            auto value = attrs.find(name::attr_name.data());
            if (value == attrs.end()) return {};
            else return value->second.timestamp();
        }

        template <typename Type>
        std::optional<uint64_t> get_attrib_timestamp_by_name(const Type &n, const std::string& att_name)
            requires(node_or_edge<Type>)
        {
            auto &attrs = n.attrs();
            auto value = attrs.find(att_name);
            if (value == attrs.end()) return {};
            else return value->second.timestamp();
        }


        template <typename name, typename Type>
        inline std::optional<decltype(name::type)> get_attrib_by_name(const Type &n)
            requires(any_node_or_edge<Type> and is_attr_name<name>) {
            using name_type = std::remove_cv_t<unwrap_reference_wrapper_t<std::remove_reference_t<std::remove_cv_t<decltype(name::type)>>>>;

            auto &attrs = n.attrs();
            auto value = attrs.find(name::attr_name.data());
            if (value == attrs.end()) return {};

            const auto &av = [&]() -> const DSR::Attribute& {
                if constexpr (node_or_edge<Type>) return value->second;
                else return value->second.read_reg();
            }();

            if constexpr (std::is_same_v< name_type, float>)
                return av.fl();
            else if constexpr (std::is_same_v< name_type, double>)
                return av.dob();
            else if constexpr (std::is_same_v< name_type, std::string>)
                return av.str();
            else if constexpr (std::is_same_v< name_type, std::int32_t>)
                return av.dec();
            else if constexpr (std::is_same_v< name_type, std::uint32_t>)
                return av.uint();
            else if constexpr (std::is_same_v< name_type, std::uint64_t>)
                return av.uint64();
            else if constexpr (std::is_same_v< name_type, bool>)
                return av.bl();
            else if constexpr (std::is_same_v< name_type, std::vector<float>>)
                return av.float_vec();
            else if constexpr (std::is_same_v< name_type, std::vector<uint8_t>>)
                return av.byte_vec();
            else if constexpr (std::is_same_v< name_type, std::vector<uint64_t>>)
                return av.u64_vec();
            else if constexpr (std::is_same_v< name_type, std::array<float, 2>>)
                return av.vec2();
            else if constexpr (std::is_same_v< name_type, std::array<float, 3>>)
                return av.vec3();
            else if constexpr (std::is_same_v< name_type, std::array<float, 4>>)
                return av.vec4();
            else if constexpr (std::is_same_v< name_type, std::array<float, 6>>)
                return av.vec6();
            else {
                []<bool flag = false>() { static_assert(flag, "Unreachable"); }();
            }
        }

        template <typename name>
        inline std::optional<std::remove_cvref_t<unwrap_reference_wrapper_t<decltype(name::type)>>> get_attrib_by_name(uint64_t id)
        requires(is_attr_name<name>)
        {
            using ret_type = std::remove_cvref_t<unwrap_reference_wrapper_t<decltype(name::type)>>;
            std::shared_lock<std::shared_mutex> lock(_mutex);
            std::optional<CRDTNode> n = get_(id);
            if (n.has_value()) {
                auto tmp = get_attrib_by_name<name>(n.value());
                if (tmp.has_value())
                {
                    if constexpr(is_reference_wrapper<decltype(name::type)>::value) {
                        return ret_type{tmp.value().get()};
                    } else {
                        return tmp;
                    }
                }
            }
            return {};
        }
        /**
         * LOCAL ATTRIBUTES MODIFICATION METHODS (for nodes and edges)
         **/
        template<typename name, typename Type, class Ta>
        inline void add_or_modify_attrib_local(Type &elem, Ta && att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta> and is_attr_name<name> and valid_type<name, Ta>())
        {

            if constexpr (std::is_same_v<Type, Node> || std::is_same_v<Type, Edge>)
            {
                Attribute at(std::forward<Ta>(att_value), get_unix_timestamp(), agent_id);
                elem.attrs().insert_or_assign(name::attr_name.data(), at);
            } else
            {
                CRDTAttribute at;
                at.value(std::forward<Ta>(att_value));
                at.timestamp(get_unix_timestamp());
                if (elem.attrs().find(name::attr_name.data()) == elem.attrs().end()) {
                    mvreg<CRDTAttribute> mv;
                    elem.attrs().insert(make_pair(name::attr_name, mv));
                }
                elem.attrs().at(name::attr_name.data()).write(at);
            }
        }

        template<typename Type, class Ta>
        inline void runtime_checked_add_or_modify_attrib_local(Type &elem, const std::string &att_name, Ta &&att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta>)
        {

            if (!attribute_types::check_type(att_name.data(), att_value)) {
                throw std::runtime_error(std::string("Invalid type in attribute ") + att_name + " - " + typeid(att_value).name() + " in: " + __FILE__  " "
                    + __FUNCTION__ + " " + std::to_string(__LINE__));
            }

            if constexpr (std::is_same_v<Type, Node> || std::is_same_v<Type, Edge>) {
                Attribute at(std::forward<Ta>(att_value), get_unix_timestamp(), agent_id);
                elem.attrs().insert_or_assign(att_name, at);
            } else {
                CRDTAttribute at;
                at.value(std::forward<Ta>(att_value));
                at.timestamp(get_unix_timestamp());
                if (elem.attrs().find(att_name) == elem.attrs().end()) {
                    mvreg<CRDTAttribute> mv;
                    elem.attrs().insert(make_pair(att_name, mv));
                }
                elem.attrs().at(att_name).write(at);
            }

        }


        template<typename name, typename Type, typename Ta>
        bool add_attrib_local(Type &elem, Ta &&att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta> and is_attr_name<name>)
        {
            if (elem.attrs().find(name::attr_name.data()) != elem.attrs().end()) return false;
            add_or_modify_attrib_local<name>(elem, std::forward<Ta>(att_value));
            return true;
        };

        template<typename Type, typename Ta>
        bool runtime_checked_add_attrib_local(Type &elem, const std::string& att_name, Ta &&att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta>)
        {
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            runtime_checked_add_or_modify_attrib_local(elem, att_name, std::forward<Ta>(att_value));
            return true;
        };

        template<typename name, typename Type>
        bool add_attrib_local(Type &elem, Attribute &attr)
            requires(any_node_or_edge<Type> and is_attr_name<name>)
        {
            if (elem.attrs().find(name::attr_name.data()) != elem.attrs().end()) return false;
            attr.timestamp(get_unix_timestamp());
            elem.attrs()[name::attr_name] = attr;
            return true;
        };

        template<typename Type>
        bool runtime_checked_add_attrib_local(Type &elem, const std::string& att_name,  Attribute &attr)
            requires(any_node_or_edge<Type>)
        {
            //TODO: Check Attribute type? Or is checked when creating and Attribute
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            attr.timestamp(get_unix_timestamp());
            elem.attrs()[att_name] = attr;
            return true;
        };

        template<typename name, typename Type, typename Ta>
        bool modify_attrib_local(Type &elem, Ta &&att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta> and is_attr_name<name>)
        {
            if (elem.attrs().find(name::attr_name.data()) == elem.attrs().end()) return false;
            add_or_modify_attrib_local<name>(elem, std::forward<Ta>(att_value));
            return true;
        };

        template<typename Type, typename Ta>
        bool runtime_checked_modify_attrib_local(Type &elem,  const std::string& att_name, Ta &&att_value)
            requires(any_node_or_edge<Type> and allowed_types<Ta>)
        {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            runtime_checked_add_or_modify_attrib_local(elem, att_name, std::forward<Ta>(att_value));
            return true;
        };

        template<typename name, typename Type>
        bool remove_attrib_local(Type &elem)
            requires(any_node_or_edge<Type> and is_attr_name<name>)
        {
            if (elem.attrs().find(name::attr_name.data()) == elem.attrs().end()) return false;
            elem.attrs().erase(name::attr_name.data());
            return true;
        }

        template< typename Type>
        bool remove_attrib_local(Type &elem, const std::string& att_name)
            requires(any_node_or_edge<Type>)
        {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            elem.attrs().erase(att_name);
            return true;
        }

    private:
        template <typename name, typename Type>
        inline auto get_crdt_attrib_by_name(const Type &n)
            requires(crdt_node_or_edge<Type> and is_attr_name<name>) //TODO: DEPRECATE THIS?
        {
            using name_type = std::remove_cv_t<unwrap_reference_wrapper_t<std::remove_reference_t<std::remove_cv_t<decltype(name::type)>>>>;

            auto &attrs = n.attrs();
            auto value = attrs.find(name::attr_name.data());

            if constexpr(is_reference_wrapper<name_type>::value) {
                using ret_type = std::optional<decltype(name_type::type)>;
                if (value == attrs.end()) return ret_type();
                auto av = *value->second.read().begin();

                if constexpr (std::is_same_v<name_type, std::reference_wrapper<const std::string>> )
                    return ret_type(av.str());
                else if constexpr (std::is_same_v<name_type, std::reference_wrapper<const std::vector<float>>>)
                    return ret_type(av.float_vec());
                else if constexpr (std::is_same_v<name_type, std::reference_wrapper<const std::vector<uint8_t>>>)
                    return ret_type(av.byte_vec());
                else if constexpr (std::is_same_v< name_type, std::reference_wrapper<const std::vector<uint64_t>>>)
                    return ret_type(av.u64_vec());
                else if constexpr (std::is_same_v< name_type, std::reference_wrapper<const std::array<float, 2>>>)
                    return ret_type(av.vec2());
                else if constexpr (std::is_same_v< name_type, std::reference_wrapper<const std::array<float, 3>>>)
                    return ret_type(av.vec3());
                else if constexpr (std::is_same_v< name_type, std::reference_wrapper<const std::array<float, 4>>>)
                    return ret_type(av.vec4());
                else if constexpr (std::is_same_v< name_type, std::reference_wrapper<const std::array<float, 6>>>)
                    return ret_type(av.vec6());
                else
                    []<bool flag = false>() { static_assert(flag, "Unreachable"); }();
            } else {
                using ret_type = std::optional<name_type>;
                if (value == attrs.end()) return ret_type();
                auto av = value->second.read_reg();

                if constexpr (std::is_same_v< name_type, float>)
                    return ret_type(av.fl());
                else if constexpr (std::is_same_v< name_type, std::int32_t>)
                    return ret_type(av.dec());
                else if constexpr (std::is_same_v< name_type, std::uint32_t>)
                    return ret_type(av.uint());
                else if constexpr (std::is_same_v< name_type, std::uint64_t>)
                    return ret_type(av.uint64());
                else if constexpr (std::is_same_v< name_type, bool>)
                    return ret_type(av.bl());
                else if constexpr (std::is_same_v< name_type, double>)
                    return ret_type(av.dob());
                else
                    []<bool flag = false>() { static_assert(flag, "Unreachable"); }();
            }

        }

    public:

        // Mixed
        inline uint64_t get_agent_id() const { return agent_id; };
        inline std::string get_agent_name() const { return agent_name; };

        void reset()
        {
            dsrparticipant.remove_participant_and_entities();

            nodes.clear();
            deleted.clear();
            name_map.clear();
            id_map.clear();
            edges.clear();
            edgeType.clear();
            nodeType.clear();
            to_edges.clear();

        }


        //////////////////////////////////////////////////////
        ///  Attribute filters
        //////////////////////////////////////////////////////
        template<typename ... Att>
        constexpr void set_ignored_attributes()
        {
            static_assert((is_attr_name<Att> && ...));
            (ignored_attributes.insert(Att::attr_name), ...);
        }

        /////////////////////////////////////////////////
        /// AUXILIARY IO SUB-API
        /////////////////////////////////////////////////
        void print()                                            { utils->print(); };
        void print_edge(const Edge &edge)                       { utils->print_edge(edge); };
        void print_node(const Node &node)                       { utils->print_node(node); };
        void print_node(uint64_t id)                            { utils->print_node(id); };
        void print_RT(uint64_t root)                      const { utils->print_RT(root); };

        void write_to_json_file(const std::string &file, const std::vector<std::string> &skip_node_content = {})  const
        {
            utils->write_to_json_file(file, skip_node_content); };

        void read_from_json_file(const std::string &file)
        {
            utils->read_from_json_file(file, [&] (const Node& node) -> std::optional<uint64_t>
            {
                bool r = false;
                {
                    std::unique_lock<std::shared_mutex> lock(_mutex);
                    if (id_map.find(node.id()) == id_map.end() and name_map.find(node.name())  == name_map.end()) {
                        std::tie(r, std::ignore) = insert_node_(user_node_to_crdt(node));
                    } else throw std::runtime_error((std::string("Cannot insert node in G, a node with the same id already exists ")
                                                     + __FILE__ + " " + " " + std::to_string(__LINE__)).data());
                }
                if (r) {
                    return node.id();
                }
                return {};
            });
        };

        //////////////////////////////////////////////////
        ///// PRIVATE COPY
        /////////////////////////////////////////////////
        std::unique_ptr<DSRGraph> G_copy();
        bool is_copy() const;


        //////////////////////////////////////////////////
        ///// Agents info
        /////////////////////////////////////////////////
        std::vector<std::string> get_connected_agents()
        {
            std::unique_lock<std::mutex> lck(participant_set_mutex);
            std::vector<std::string> ret_vec(participant_set.size());
            for (auto &[k, _]: participant_set)
            {
                ret_vec.emplace_back(k);
            }
            return ret_vec;
        }

    private:

        DSRGraph(const DSRGraph& G); //Private constructor for DSRCopy

        Nodes nodes;
        mutable std::shared_mutex _mutex;
        mutable std::shared_mutex _mutex_cache_maps;
        mutable std::mutex mtx_entity_creation;

        const uint32_t agent_id;
        const std::string agent_name;
        const bool copy;
        std::unique_ptr<Utilities> utils;
        std::unordered_set<std::string_view> ignored_attributes;
        ThreadPool tp, tp_delta_attr;
        bool same_host;
        id_generator generator;

        //////////////////////////////////////////////////////////////////////////
        // Cache maps
        ///////////////////////////////////////////////////////////////////////////

        std::unordered_set<uint64_t> deleted;     // deleted nodes, used to avoid insertion after remove.
        std::unordered_map<std::string, uint64_t> name_map;     // mapping between name and id of nodes.
        std::unordered_map<uint64_t, std::string> id_map;       // mapping between id and name of nodes.
        std::unordered_map<std::pair<uint64_t, uint64_t>, std::unordered_set<std::string>, hash_tuple> edges;      // collection with all graph edges. ((from, to), key)
        std::unordered_map<uint64_t , std::unordered_set<std::pair<uint64_t, std::string>,hash_tuple>> to_edges;      // collection with all graph edges. (to, (from, key))
        std::unordered_map<std::string, std::unordered_set<std::pair<uint64_t, uint64_t>, hash_tuple>> edgeType;  // collection with all edge types.
        std::unordered_map<std::string, std::unordered_set<uint64_t>> nodeType;  // collection with all node types.

        void update_maps_node_delete(uint64_t id, const std::optional<CRDTNode>& n);
        void update_maps_node_insert(uint64_t id, const CRDTNode &n);
        void update_maps_edge_delete(uint64_t from, uint64_t to, const std::string &key = "");
        void update_maps_edge_insert(uint64_t from, uint64_t to, const std::string &key);


        //////////////////////////////////////////////////////////////////////////
        // Non-blocking graph operations
        //////////////////////////////////////////////////////////////////////////
        std::optional<CRDTNode> get_(uint64_t id);
        std::optional<CRDTEdge> get_edge_(uint64_t from, uint64_t to, const std::string &key);
        std::tuple<bool, std::optional<IDL::MvregNode>> insert_node_(CRDTNode &&node);
        std::tuple<bool, std::optional<std::vector<IDL::MvregNodeAttr>>> update_node_(CRDTNode &&node);
        std::tuple<bool, std::vector<std::tuple<uint64_t, uint64_t, std::string>>, std::optional<IDL::MvregNode>, std::vector<IDL::MvregEdge>> delete_node_(uint64_t id);
        std::optional<IDL::MvregEdge> delete_edge_(uint64_t from, uint64_t t, const std::string &key);
        std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>> insert_or_assign_edge_(CRDTEdge &&attrs, uint64_t from, uint64_t to);

        //////////////////////////////////////////////////////////////////////////
        // Other methods
        //////////////////////////////////////////////////////////////////////////
        std::map<uint64_t , IDL::MvregNode> Map();

        //////////////////////////////////////////////////////////////////////////
        // CRDT join operations
        ///////////////////////////////////////////////////////////////////////////
        void join_delta_node(IDL::MvregNode &&mvreg);
        void join_delta_edge(IDL::MvregEdge &&mvreg);
        std::optional<std::string> join_delta_node_attr(IDL::MvregNodeAttr &&mvreg);
        std::optional<std::string> join_delta_edge_attr(IDL::MvregEdgeAttr &&mvreg);
        void join_full_graph(IDL::OrMap &&full_graph);

        bool process_delta_edge(uint64_t from, uint64_t to, const std::string& type, mvreg<CRDTEdge> && delta);
        void process_delta_node_attr(uint64_t id, const std::string& att_name, mvreg<CRDTAttribute> && attr);
        void process_delta_edge_attr(uint64_t from, uint64_t to, const std::string& type, const std::string& att_name, mvreg<CRDTAttribute> && attr);

        //Maps for temporary deltas
        std::unordered_multimap<uint64_t, std::tuple<std::string, mvreg<DSR::CRDTAttribute>, uint64_t> > unprocessed_delta_node_att;
        std::unordered_multimap<uint64_t, std::tuple<uint64_t, std::string, mvreg<DSR::CRDTEdge>, uint64_t>> unprocessed_delta_edge_from;
        std::unordered_multimap<uint64_t, std::tuple<uint64_t, std::string, mvreg<DSR::CRDTEdge>, uint64_t>> unprocessed_delta_edge_to;
        std::unordered_multimap<std::tuple<uint64_t, uint64_t, std::string>, std::tuple<std::string, mvreg<DSR::CRDTAttribute>, uint64_t>, hash_tuple> unprocessed_delta_edge_att;

        //Custom function for each rtps topic
        class NewMessageFunctor {
        public:
            DSRGraph *graph{};
            std::function<void(eprosima::fastdds::dds::DataReader* reader, DSR::DSRGraph *graph)> f;

            NewMessageFunctor(DSRGraph *graph_,
                              std::function<void(eprosima::fastdds::dds::DataReader* reader,  DSR::DSRGraph *graph)> f_)
                    : graph(graph_), f(std::move(f_)) {}

            NewMessageFunctor() = default;

            void operator()(eprosima::fastdds::dds::DataReader* reader) const { f(reader, graph); };
        };

        //Custom function for each rtps topic
        class ParticipantChangeFunctor {
        public:
            DSRGraph *graph{};
            std::function<void(DSRGraph *graph_,eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)> f;

            ParticipantChangeFunctor(DSRGraph *graph_,
                                     std::function<void(DSRGraph *graph_, eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)> f_)
                    : graph(graph_), f(std::move(f_)) {}

            ParticipantChangeFunctor() = default;

            void operator()(eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) const
            {
                f(graph, std::forward<eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&>(info));
            };
        };


        //Threads handlers
        std::pair<bool, bool> start_fullgraph_request_thread();
        void start_fullgraph_server_thread();
        void start_subscription_threads(bool showReceived);
        void node_subscription_thread(bool showReceived);
        void edge_subscription_thread(bool showReceived);
        void node_attrs_subscription_thread(bool showReceived);
        void edge_attrs_subscription_thread(bool showReceived);
        void fullgraph_server_thread();
        std::pair<bool, bool> fullgraph_request_thread();


        // RTSP participant
        //TODO: Move this to a class?
        DSRParticipant dsrparticipant;
        std::unordered_map<std::string, bool> participant_set;

        mutable std::mutex participant_set_mutex;

        DSRPublisher dsrpub_node;
        DSRSubscriber dsrsub_node;
        NewMessageFunctor dsrpub_call_node;

        DSRPublisher dsrpub_edge;
        DSRSubscriber dsrsub_edge;
        NewMessageFunctor dsrpub_call_edge;

        DSRPublisher dsrpub_node_attrs;
        DSRSubscriber dsrsub_node_attrs;
        NewMessageFunctor dsrpub_call_node_attrs;

        DSRPublisher dsrpub_edge_attrs;
        DSRSubscriber dsrsub_edge_attrs;
        NewMessageFunctor dsrpub_call_edge_attrs;

        DSRSubscriber dsrsub_graph_request;
        DSRPublisher dsrpub_graph_request;
        NewMessageFunctor dsrpub_graph_request_call;

        DSRSubscriber dsrsub_request_answer;
        DSRPublisher dsrpub_request_answer;
        NewMessageFunctor dsrpub_request_answer_call;

    signals:
        void update_node_signal(uint64_t, const std::string &type/*, uint64_t=0*/);
        void update_node_attr_signal(uint64_t id ,const std::vector<std::string>& att_names/*, uint64_t=0*/);

        void update_edge_signal(uint64_t from, uint64_t to, const std::string &type/*, uint64_t=0*/);
        void update_edge_attr_signal(uint64_t from, uint64_t to, const std::string &type, const std::vector<std::string>& att_name/*, uint64_t=0*/);

        void del_edge_signal(uint64_t from, uint64_t to, const std::string &edge_tag/*, uint64_t=0*/);
        void del_node_signal(uint64_t id/*, uint64_t=0*/) ;

    };
} // namespace CRDT

#endif
