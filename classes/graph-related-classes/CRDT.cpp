//
// Created by crivac on 5/02/19.
//

#include "CRDT.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>

using namespace CRDT;

/////////////////////////////////////////////////
///// PUBLIC METHODS
/////////////////////////////////////////////////

CRDTGraph::CRDTGraph(int root, std::string name, int id, std::string dsr_input_file) : agent_id(id) , agent_name(name)
{
    graph_root = root;
    nodes = Nodes(graph_root);
    utils = std::make_unique<Utilities>(this);
    std::cout << "Agent name: " << agent_name << std::endl;
    work = true;

    // RTPS Create participant 
    auto [suc, participant_handle] = dsrparticipant.init();

    // RTPS Initialize publisher with general topic
    dsrpub.init(participant_handle, "DSR", dsrparticipant.getDSRTopicName());
    dsrpub_graph_request.init(participant_handle, "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName());
    dsrpub_request_answer.init(participant_handle, "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName());

    // RTPS Initialize comms threads
     if(dsr_input_file != std::string())
    {
        try
        {   
            utils->read_from_json_file(dsr_input_file); 
            qDebug() << __FUNCTION__ << "Warning, graph read from file " << QString::fromStdString(dsr_input_file);     
        }
        catch(const CRDT::DSRException& e)
        {  
            std::cout << e.what() << '\n';  
            qFatal("Aborting program. Cannot continue without intial file");
        }
        start_fullgraph_server_thread();
        start_subscription_thread(false);
    }
    else
    {    
        start_subscription_thread(false);     // regular subscription to deltas
        bool res = start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
        if(res == false) {
            eprosima::fastrtps::Domain::removeParticipant(participant_handle); // Remove a Participant and all associated publishers and subscribers.
            qFatal("CRDTGraph aborting: could not get DSR from the network after timeout");  //JC ¿se pueden limpiar aquí cosas antes de salir?

        }
    }
    qDebug() << __FUNCTION__ << "Constructor finished OK";
}

CRDTGraph::~CRDTGraph() 
{
    qDebug() << "Removing rtps participant";
    eprosima::fastrtps::Domain::removeParticipant(dsrparticipant.getParticipant()); // Remove a Participant and all associated publishers and subscribers.
    fullgraph_thread.join();
    delta_thread.join();
}

//////////////////////////////////////
/// NODE METHODS
/////////////////////////////////////

std::optional<Node> CRDTGraph::get_node(const std::string& name)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    if (name.empty()) return {};
    int id = get_id_from_name(name).value_or(-1);
    if (in(id)) {
        auto n = &nodes[id].dots().ds.rbegin()->second;
        if (n->name() == name) return Node(*n);
    }

    return {};
}

std::optional<Node> CRDTGraph::get_node(int id)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}


bool CRDTGraph::insert_or_assign_node(const N &node)
{
    if (node.id() == -1) return false;
    bool r;
    std::optional<AworSet> aw;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if ((id_map.find(node.id()) != id_map.end() and id_map[node.id()] != node.name())
            or ( name_map.find(node.name())  != name_map.end() and name_map[node.name()] != node.id() ))
            throw std::runtime_error((std::string("Cannot insert node in G, id mut be unique ") + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        std::tie(r, aw) = insert_or_assign_node_(node);
    }
    if (r) {
        if (aw.has_value())
                dsrpub.write(&aw.value());

        emit update_node_signal(node.id(), node.type());
        for (const auto &[k,v]: node.fano())
            emit update_edge_signal(node.id(), k.to(), v.type());
    }
    return r;
}


std::pair<bool, std::optional<AworSet>> CRDTGraph::insert_or_assign_node_(const N &node)
{
    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].dots().ds.empty() and nodes[node.id()].dots().ds.rbegin()->second == node) {
            return {true, {} };
        }
        aworset<Node, int> delta = nodes[node.id()].add(node, node.id());
        update_maps_node_insert(node.id(), node);

        return { true, translateAwCRDTtoIDL(node.id(), delta) };
    }
    return {false, {} };
}

std::optional<uint32_t> CRDTGraph::insert_node(const Node& node) {
    if (node.id() == -1) return {};
    std::optional<AworSet> aw;
    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (id_map.find(node.id()) == id_map.end() and name_map.find(node.name())  == name_map.end()) {
            //TODO: Poner id con el proxy y generar el nombre
            std::tie(r, aw) = insert_or_assign_node_(node);
        } else throw std::runtime_error((std::string("Cannot insert node in G, a node with the same id already exists ")
                                         + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
    }
    if (r) {
        if (aw.has_value())
            dsrpub.write(&aw.value());

        emit update_node_signal(node.id(), node.type());
        for (const auto &[k,v]: node.fano())
                emit update_edge_signal(node.id(), k.to(), v.type());

        return node.id();
    }
    return {};
}

bool CRDTGraph::update_node(const N &node)
{
    if (node.id() == -1) return false;
    bool r = false;
    std::optional<AworSet> aw;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if ((id_map.find(node.id()) != id_map.end() and id_map[node.id()] != node.name())  or (name_map.find(node.name()) != name_map.end() and name_map[node.name()] != node.id()))
            throw std::runtime_error((std::string("Cannot update node in G, id and name must be unique")  + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        std::tie(r, aw) = insert_or_assign_node_(node);
    }
    if (r) {
        if (aw.has_value())
            dsrpub.write(&aw.value());

        emit update_node_signal(node.id(), node.type());
    }
    return r;
}


bool CRDTGraph::delete_node(const std::string& name)
{
    bool result = false;
    vector<tuple<int,int, std::string>> edges_;
    vector<AworSet> aw_;

    std::optional<int> id = {};
    {
        std::unique_lock<std::shared_mutex>  lock(_mutex);
        id = get_id_from_name(name);
        if(id.has_value()) {
            std::tie(result, edges_, aw_) = delete_node_(id.value());
        } else {
            return false;
        }
    }

    if (result) {
        emit del_node_signal(id.value());

        for (auto &a  : aw_)
            dsrpub.write(&a);

        for (auto &[id0, id1, label] : edges_)
                emit del_edge_signal(id0, id1, label);
        return true;
    }
    return false;
}

bool CRDTGraph::delete_node(int id)
{
    bool result;
    vector<tuple<int,int, std::string>> edges_;
    vector<AworSet> aw_;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(id)) {
            std::tie(result, edges_, aw_) = delete_node_(id);
        } else { return false; }
    }

    if (result) {
        emit del_node_signal(id);

        for (auto &a  : aw_)
            dsrpub.write(&a);
        for (auto &[id0, id1, label] : edges_)
            emit del_edge_signal(id0, id1, label);

        return true;
    }
    return false;
}

std::tuple<bool, vector<tuple<int, int, std::string>>, vector<AworSet>> CRDTGraph::delete_node_(int id)
{
    vector<tuple<int,int, std::string>> edges_;
    vector<AworSet> aw;

    //1. Get and remove node.
    auto node = get_(id);
    if (!node.has_value()) return make_tuple(false, edges_, aw);
    for (const auto &v : node.value().fano()) { // Delete all edges from this node.
        std::cout << id << " -> " << v.first.to() << " " << v.first.type() << std::endl;
         edges_.emplace_back(make_tuple(id, v.first.to(), v.first.type()));
    }
    // Get remove delta.
    auto delta = nodes[id].rmv(nodes[id].dots().ds.rbegin()->second);
    aw.emplace_back(translateAwCRDTtoIDL(id, delta));
    update_maps_node_delete(id, node.value());

    //2. search and remove edges.
    //For each node check if there is an edge to remove.
    for (auto &[k, v] : nodes.getMapRef()) {
        if (edges.find({k, id}) == edges.end()) continue;
        // Remove all edges between them
        auto visited_node =  Node(v.dots().ds.rbegin()->second);
        for (const auto &key : edges[{k, id}]) {
            EdgeKey ek; ek.to(k); ek.type(key);
            visited_node.fano().erase(ek);
            edges_.emplace_back(make_tuple(visited_node.id(), id, key));

            edgeType[key].erase({visited_node.id(), id});
        }

        auto delta = nodes[visited_node.id()].add(visited_node, visited_node.id());
        aw.emplace_back( translateAwCRDTtoIDL(visited_node.id(), delta));

        //Remove all from cache
        edges.erase({visited_node.id(), id});
    }
    return make_tuple(true,  edges_, aw);
}

std::vector<Node> CRDTGraph::get_nodes_by_type(const std::string& type)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Node> nodes_;
    if (nodeType.find(type) != nodeType.end()) {
        for (auto id: nodeType[type]) {
            auto n = get_(id);
            if (n.has_value())
                nodes_.emplace_back(n.value());
        }
    }
    return nodes_;
}

//////////////////////////////////////////////////////////////////////////////////
// EDGE METHODS
//////////////////////////////////////////////////////////////////////////////////
std::optional<Edge> CRDTGraph::get_edge(const std::string& from, const std::string& to, const std::string& key)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::optional<int> id_from = get_id_from_name(from);
    std::optional<int> id_to = get_id_from_name(to);
    if (id_from.has_value() and id_to.has_value())
        return get_edge_(id_from.value(), id_to.value(), key);
    return {};
}

std::optional<Edge> CRDTGraph::get_edge(int from, int to, const std::string &key)
{
    return get_edge_(from, to, key);
}

std::optional<Edge> CRDTGraph::get_edge_(int from, int  to, const std::string& key)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    if (in(from) && in(to)) {
        auto n = get_(from);
        if (n.has_value()) {
            EdgeKey ek; ek.to(to); ek.type(key);
            auto edge = n.value().fano().find(ek);
            if (edge != n.value().fano().end()) {
                return Edge(edge->second);
            }
        }
         std::cout << __FUNCTION__ <<":" << __LINE__ << " Error obteniedo edge from: "<< from  << " to: " << to <<" key " << key << endl;
     }
     return {};
}

bool CRDTGraph::insert_or_assign_edge(const Edge& attrs)
{
    bool r = false;
    std::optional<AworSet> aw;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        int from = attrs.from();
        int to = attrs.to();
        if (in(from) && in(to))
        {
            auto node = get_(from);
            if (node.has_value()) {
                EdgeKey ek; ek.to(to); ek.type(attrs.type());
                node.value().fano().insert_or_assign(ek, attrs);
                node.value().agent_id(agent_id);
                std::tie(r, aw) = insert_or_assign_node_(node.value());
            }
        } else
        {
            std::cout << __FUNCTION__ <<":" << __LINE__ <<" Error. ID:"<<from<<" or "<<to<<" not found. Cant update. "<< std::endl;
            return false;
        }
    }
    if (r)
        emit update_edge_signal( attrs.from(),  attrs.to(), attrs.type());
    if (aw.has_value())
        dsrpub.write(&aw.value());

    return true;
}

void CRDTGraph::insert_or_assign_edge_RT(Node& n, int to, std::vector<float>&& trans, std::vector<float>&& rot_euler)
{
    bool r = false;

    std::optional<AworSet> awor1;
    std::optional<AworSet> awor2;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to))
        {
            EdgeKey ek; ek.to(to); ek.type("RT");   
            Edge e; e.to(to); e.from(n.id()); e.type("RT");
            Attrib tr; tr.type(3); tr.value().float_vec(trans);
            Attrib rot; rot.type(3); rot.value().float_vec(rot_euler);
            e.attrs().insert_or_assign("rotation_euler_xyz", rot);
            e.attrs().insert_or_assign("translation", tr);
            n.fano().insert_or_assign(ek, e);
            n.agent_id(agent_id);
            Node to_n = get_(to).value();
            bool res1 = modify_attrib(to_n, "parent", n.id());
            if (!res1) (void) add_attrib(to_n, "parent", n.id());
            bool res2 = modify_attrib(to_n, "level",  get_node_level(n).value() + 1 );
            if (!res2) (void) add_attrib(to_n, "level",  get_node_level(n).value() + 1 );

            auto [r1, aw1] = insert_or_assign_node_(n);
            auto [r2, aw2] = insert_or_assign_node_(to_n);

            if(r1 and r2) {
                r = true;
                awor1 = std::move(aw1);
                awor2 = std::move(aw2);
            }
            else
                throw std::runtime_error("Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
        } else
            throw std::runtime_error("Destination node " + std::to_string(n.id()) + " not found in G in insert_or_assign_edge_RT() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (r)
        emit update_edge_signal( n.id(),  to, "RT");

    if (awor1.has_value()) {  dsrpub.write(&awor1.value()); }
    if (awor2.has_value()) {  dsrpub.write(&awor2.value()); }

}

void CRDTGraph::insert_or_assign_edge_RT(Node& n, int to, const std::vector<float>& trans, const std::vector<float>& rot_euler)
{
    bool r = false;

    std::optional<AworSet> awor1;
    std::optional<AworSet> awor2;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to))
        {
            EdgeKey ek; ek.to(to); ek.type("RT");   // PONER EN UNA TABLA
            Edge e; e.to(to); e.from(n.id()); n.type("RT");
            Attrib tr; tr.type(3); tr.value().float_vec(trans);
            Attrib rot; rot.type(3); rot.value().float_vec(rot_euler);
            e.attrs().insert_or_assign("rotation_euler_xyz", rot);
            e.attrs().insert_or_assign("translation", tr);
            n.fano().insert_or_assign(ek, e);
            n.agent_id(agent_id);
            Node to_n = get_(to).value();
            bool res1 = modify_attrib(to_n, "parent", n.id());
            if (!res1) (void) add_attrib(to_n, "parent", n.id());
            bool res2 = modify_attrib(to_n, "level",  get_node_level(n).value() + 1 );
            if (!res2) (void) add_attrib(to_n, "level",  get_node_level(n).value() + 1 );

            auto [r1, aw1] = insert_or_assign_node_(n);
            auto [r2, aw2] = insert_or_assign_node_(to_n);

            if(r1 and r2) {
                r = true;
                awor1 = std::move(aw1);
                awor2 = std::move(aw2);
            }
            else
                throw std::runtime_error("Could not insert node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
        } else
            throw std::runtime_error("Destination node " + std::to_string(n.id()) + " not found in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (r)
        emit update_edge_signal( n.id(),  to, "RT");

    if (awor1.has_value()) {  dsrpub.write(&awor1.value()); }
    if (awor2.has_value()) {  dsrpub.write(&awor2.value()); }

}

bool CRDTGraph::delete_edge(int from, int to, const std::string& key)
{
    bool result;
    std::optional<AworSet> aw;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (!in(from) || !in(to)) return false;
        std::tie(result, aw) = delete_edge_(from, to, key);
    }
    if (result)
            emit del_edge_signal(from, to, key);
    if (aw.has_value())
        dsrpub.write(&aw.value());


    return result;
}

bool CRDTGraph::delete_edge(const std::string& from, const std::string& to, const std::string& key)
{
    std::optional<int> id_from = {};
    std::optional<int> id_to = {};
    std::optional<AworSet> aw;
    bool result = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id_from = get_id_from_name(from);
        id_to = get_id_from_name(to);

        if (id_from.has_value() && id_to.has_value()) {
            std::tie(result, aw) = delete_edge_(id_from.value(), id_to.value(), key);
        }
    }

    if (result)
        emit del_edge_signal(id_from.value(), id_to.value(), key);
    if (aw.has_value())
        dsrpub.write(&aw.value());


    return result;
}

std::pair<bool, std::optional<AworSet>> CRDTGraph::delete_edge_(int from, int to, const std::string& key)
{
    auto node = get_(from);
    if (node.has_value()) {
        EdgeKey ek;
        ek.to(to);
        ek.type(key);
        if (node.value().fano().find(ek) != node.value().fano().end()) {
            node.value().fano().erase(ek);
            update_maps_edge_delete(from, to, key);
            node.value().agent_id(agent_id);
            return insert_or_assign_node_(node.value());
        }
    }
    return { false, {} };
}

std::vector<Edge> CRDTGraph::get_edges_by_type(const Node& node, const std::string& type)
{
    std::vector<Edge> edges_;
    for (auto &[key, edge] : node.fano()) 
        if( key.type() == type )
            edges_.emplace_back(edge);
    return edges_;
}

std::vector<Edge> CRDTGraph::get_edges_by_type(const std::string& type)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Edge> edges_;
    if (edgeType.find(type) != edgeType.end())
    {
        for (auto &[from, to] : edgeType[type]) 
        {
            auto n = get_edge_(from, to, type);
            if (n.has_value())
                edges_.emplace_back(n.value());
        }
    }
    return edges_;
}

std::vector<Edge> CRDTGraph::get_edges_to_id(int id) {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Edge> edges_;
    for (const auto &[key, types] : edges)
    {
        auto [from, to] = key;
        if (to == id) {
            for (const std::string& type : types) {
                auto n = get_edge_(from, to, type);
                if (n.has_value())
                    edges_.emplace_back(n.value());
            }
        }
    }
    return edges_;
}

std::optional<std::map<EdgeKey, Edge>> CRDTGraph::get_edges(int id) 
{ 
    std::optional<Node> n = get_node(id);
    return n.has_value() ?  std::optional<std::map<EdgeKey, Edge>>(n.value().fano()) : std::nullopt;
};

Edge CRDTGraph::get_edge_RT(const Node &n, int to)
{
    auto edges = n.fano();
    EdgeKey key; key.to(to); key.type("RT");
    auto res  = edges.find(key);
    if (res != edges.end())
        return res->second;
    else
        throw std::runtime_error("Could not find edge " + std::to_string(key.to()) + " in node " + std::to_string(n.id()) + " in edge_to_RTMat() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    
}

RTMat CRDTGraph::get_edge_RT_as_RTMat(const Edge &edge)
{
    auto r = get_attrib_by_name<std::vector<float>>(edge, "rotation_euler_xyz");
    auto t = get_attrib_by_name<std::vector<float>>(edge, "translation");
    if( r.has_value() and t.has_value() )
        return RTMat { r.value()[0], r.value()[1], r.value()[2], t.value()[0], t.value()[1], t.value()[2] } ;
    else
        throw std::runtime_error("Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) + " in get_edge_RT as_RTMat() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
   
}

 RTMat CRDTGraph::get_edge_RT_as_RTMat(Edge &&edge)
 {
    auto r = get_attrib_by_name<std::vector<float>>(edge, "rotation_euler_xyz");
    auto t = get_attrib_by_name<std::vector<float>>(edge, "translation");
    if( r.has_value() and t.has_value() )
        return RTMat { r.value()[0], r.value()[1], r.value()[2], t.value()[0], t.value()[1], t.value()[2] } ;
    else
        throw std::runtime_error("Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) + " in get_edge_RT as_RTMat() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
 } 
     

/////////////////////////////////////////////////
///// Utils
/////////////////////////////////////////////////

std::map<long,Node> CRDTGraph::getCopy() const
{
    std::map<long,Node> mymap;
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    for (auto &[key, val] : nodes.getMap())
        mymap[key] = val.dots().ds.rbegin()->second;

    return mymap;
}

 std::vector<long> CRDTGraph::getKeys() const
{
    std::vector<long> keys;
    std::shared_lock<std::shared_mutex>  lock(_mutex);

    for (auto &[key, val] : nodes.getMap())
        keys.emplace_back(key);

    return keys;
}


//////////////////////////////////////////////////////////////////////////////
/////  CORE
//////////////////////////////////////////////////////////////////////////////

std::optional<Node> CRDTGraph::get(int id) {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}

std::optional<Node> CRDTGraph::get_(int id) {

    if (in(id)) {
        if (!nodes[id].dots().ds.empty()) {
            return nodes[id].dots().ds.rbegin()->second;
        }
    }
    return {};
}

std::optional<std::int32_t> CRDTGraph::get_node_level(Node& n)
{
    return get_attrib_by_name<std::int32_t>(n, "level");
}

std::optional<std::int32_t> CRDTGraph::get_node_parent(const Node& n)
{
    return get_attrib_by_name<std::int32_t>(n, "parent");
}

std::optional<std::int32_t> CRDTGraph::get_parent_id(const Node& n)
{
    return get_attrib_by_name<std::int32_t>(n, "parent");
}

std::optional<Node> CRDTGraph::get_parent_node(const Node &n)
{
    auto p =  get_attrib_by_name<std::int32_t>(n, "parent");
    if (p.has_value()) {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        return get_(p.value());
    }
    return {};
}


std::string CRDTGraph::get_node_type(Node& n)
{
    //try {
        return n.type();
    //} catch(const std::exception &e){
    //    std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};
    //return "error";
}

inline void CRDTGraph::update_maps_node_delete(int id, const Node& n)
{
    nodes.erase(id);
    name_map.erase(id_map[id]);
    id_map.erase(id);
    deleted.insert(id);

    if (nodeType.find(n.type()) != nodeType.end())
        nodeType[n.type()].erase(id);

    for (const auto &[k,v] : n.fano()) {
        edges[{id, v.to()}].erase(k.type());
        if(edges[{id,k.to()}].empty()) edges.erase({id,k.to()});
        edgeType[k.type()].erase({id, k.to()});
    }
}

inline void CRDTGraph::update_maps_node_insert(int id, const Node& n)
{
    name_map[n.name()] = id;
    id_map[id] = n.name();
    nodeType[n.type()].emplace(id);

    for (const auto &[k,v] : n.fano()) {
        edges[{id, k.to()}].insert(k.type());
        edgeType[k.type()].insert({id, k.to()});
    }
}


inline void CRDTGraph::update_maps_edge_delete(int from, int to, const std::string& key)
{
    edges[{from, to}].erase(key);
    if(edges[{from, to}].empty()) edges.erase({from, to});
    edgeType[key].erase({from, to});
}


std::optional<int> CRDTGraph::get_id_from_name(const std::string &name)
{
        auto v = name_map.find(name);
        if (v != name_map.end()) return v->second;
        return {};
}

std::optional<std::string> CRDTGraph::get_name_from_id(std::int32_t id)
{
    auto v = id_map.find(id);
    if (v != id_map.end()) return v->second;
    return {};
}

size_t CRDTGraph::size ()
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return nodes.getMapRef().size();
};

bool CRDTGraph::in(const int &id) const
{
    return nodes.in(id);
}

bool CRDTGraph::empty(const int &id)
{
    if (nodes.in(id)) 
    {
        return nodes[id].dots().ds.empty();
    } else
        return false;
}

void CRDTGraph::join_delta_node(AworSet aworSet)
{
    try{
        //vector<tuple<int, int, std::string>> remove;
        bool signal = false;
        auto d = translateAwIDLtoCRDT(aworSet);
        Node nd;
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            if (deleted.find(aworSet.id()) == deleted.end()) {

                (nodes[aworSet.id()].dots().ds.rbegin() != nodes[aworSet.id()].dots().ds.rend()) ?
                    nd = nodes[aworSet.id()].dots().ds.rbegin()->second : Node();

                nodes[aworSet.id()].join_replace(d);
                if (nodes[aworSet.id()].dots().ds.size() == 0 or aworSet.dk().ds().size() == 0) {
                    update_maps_node_delete(aworSet.id(), nd);
                } else {
                    signal = true;
                    update_maps_node_insert(aworSet.id(), nodes[aworSet.id()].dots().ds.rbegin()->second);
                }
            }
        }

        if (signal) {
            //check what change is joined
            if (nd.attrs() != nodes[aworSet.id()].dots().ds.rbegin()->second.attrs()) {
                emit update_node_signal(aworSet.id(), nodes[aworSet.id()].dots().ds.rbegin()->second.type());
            } else {
                std::map<EdgeKey, Edge> diff_remove;
                std::set_difference(nd.fano().begin(), nd.fano().end(),
                              nodes[aworSet.id()].dots().ds.rbegin()->second.fano().begin(),
                              nodes[aworSet.id()].dots().ds.rbegin()->second.fano().end(),
                                    std::inserter(diff_remove, diff_remove.begin()));
                std::map<EdgeKey, Edge> diff_insert;
                std::set_difference(nodes[aworSet.id()].dots().ds.rbegin()->second.fano().begin(),
                                    nodes[aworSet.id()].dots().ds.rbegin()->second.fano().end(),
                                    nd.fano().begin(), nd.fano().end(),
                                    std::inserter(diff_insert, diff_insert.begin()));

                for (const auto &[k,v] : diff_remove)
                        emit del_edge_signal(aworSet.id(), k.to(), k.type());

                for (const auto &[k,v] : diff_insert) {
                    emit update_edge_signal(aworSet.id(), k.to(), k.type());
                }
            }
        }
        else {
            emit del_node_signal(aworSet.id());
        }

    } catch(const std::exception &e){std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};
}

void CRDTGraph::join_full_graph(OrMap full_graph) 
{
    vector<tuple<bool, int, std::string, Node>> updates;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        auto m = static_cast<map<int, int>>(full_graph.cbase().cc());
        std::set<pair<int, int>> s;
        for (auto &v : full_graph.cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));

        for (auto &[k, val] : full_graph.m())
        {
            auto awor = translateAwIDLtoCRDT(val);
            Node nd = (nodes[k].dots().ds.rbegin() == nodes[k].dots().ds.rend()) ? Node() : nodes[k].dots().ds.rbegin()->second;

            if (deleted.find(k) == deleted.end()) {
                nodes[k].join_replace(awor);
                if (awor.dots().ds.size() == 0) {
                    update_maps_node_delete(k, nd);
                    updates.emplace_back(make_tuple(false, k, "", nd));
                } else {
                    if (!nodes[k].dots().ds.empty()) {
                        update_maps_node_insert(k, awor.dots().ds.begin()->second);
                        updates.emplace_back(make_tuple(true, k, nodes[k].dots().ds.begin()->second.type(), nd));
                    } else {
                        update_maps_node_delete(k, nd);
                    }
                }
            }
        }


    }
    for (auto &[signal, id, type, nd] : updates)
        if (signal) {
            //check what change is joined
            if (nd.attrs() != nodes[id].dots().ds.rbegin()->second.attrs()) {
                emit update_node_signal(id, nodes[id].dots().ds.rbegin()->second.type());
            } else {
                std::map<EdgeKey, Edge> diff_remove;
                std::set_difference(nd.fano().begin(), nd.fano().end(),
                                    nodes[id].dots().ds.rbegin()->second.fano().begin(),
                                    nodes[id].dots().ds.rbegin()->second.fano().end(),
                                    std::inserter(diff_remove, diff_remove.begin()));
                std::map<EdgeKey, Edge> diff_insert;
                std::set_difference(nodes[id].dots().ds.rbegin()->second.fano().begin(),
                                    nodes[id].dots().ds.rbegin()->second.fano().end(),
                                    nd.fano().begin(), nd.fano().end(),
                                    std::inserter(diff_insert, diff_insert.begin()));

                for (const auto &[k,v] : diff_remove)
                        emit del_edge_signal(id, k.to(), k.type());

                for (const auto &[k,v] : diff_insert) {
                    emit update_edge_signal(id, k.to(), k.type());
                }
            }
        }
        else {
            emit del_node_signal(id);
        }
}

bool CRDTGraph::start_fullgraph_request_thread() 
{
    return fullgraph_request_thread();
}

void CRDTGraph::start_fullgraph_server_thread() 
{
    fullgraph_thread = std::thread(&CRDTGraph::fullgraph_server_thread, this);
}

void CRDTGraph::start_subscription_thread(bool showReceived) 
{
    delta_thread = std::thread(&CRDTGraph::subscription_thread, this, showReceived);
}

int CRDTGraph::id() 
{
    return nodes.getId();
}

DotContext CRDTGraph::context() 
{
    DotContext om_dotcontext;
    for (auto &kv_cc : nodes.context().getCcDc().first) 
    {
        om_dotcontext.cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : nodes.context().getCcDc().second)
    {
        PairInt p_i;
        p_i.first(kv_dc.first);
        p_i.second(kv_dc.second);
        om_dotcontext.dc().push_back(p_i);
    }
    return om_dotcontext;
}

std::map<int,AworSet> CRDTGraph::Map() 
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::map<int,AworSet>  m;
    for (auto kv : nodes.getMapRef()) 
    { 
        aworset<Node, int> n;

        auto last = *kv.second.dots().ds.rbegin();
        n.dots().ds.insert(last);
        n.dots().c = kv.second.dots().c;
        m[kv.first] = translateAwCRDTtoIDL(kv.first, n);
    }
    return m;
}

void CRDTGraph::subscription_thread(bool showReceived) 
{
	 // RTPS Initialize subscriptor
    auto lambda_general_topic = [&] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                AworSet sample;
                //std::cout << "Unreaded: " << sub->get_unread_count() << std::endl;
                //read or take?
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                            if (showReceived)  std::cout << " Received:" << sample.id() << " node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call = NewMessageFunctor(this, &work, lambda_general_topic);
	auto res = dsrsub.init(dsrparticipant.getParticipant(), "DSR", dsrparticipant.getDSRTopicName(), dsrpub_call);
    std::cout << (res == true ? "Ok" : "Error") << std::endl;
}

void CRDTGraph::fullgraph_server_thread() 
{
    std::cout << __FUNCTION__ << "->Entering thread to attend full graph requests" << std::endl;
    // Request Topic
    auto lambda_graph_request = [&] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        GraphRequest sample;
        //readNextData o takeNextData
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                    std::cout << " Received Full Graph request: from " << m_info.sample_identity.writer_guid()
                              << std::endl;
                    *work = false;
                    OrMap mp;
                    mp.id(graph->id());
                    mp.m(graph->Map());
                    mp.cbase(graph->context());
                    std::cout << "nodos enviados: " << mp.m().size()  << std::endl;

                    dsrpub_request_answer.write(&mp);

                    for (auto &[k, v] : Map())
                        std::cout << k << "," << v.dk() << std::endl;
                    std::cout << "Full graph written" << std::endl;
                    *work = true;
                }
            }
        }
    };
    dsrpub_graph_request_call = NewMessageFunctor(this, &work, lambda_graph_request);
    dsrsub_graph_request.init(dsrparticipant.getParticipant(), "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName(), dsrpub_graph_request_call);
};

bool CRDTGraph::fullgraph_request_thread() 
{
    bool sync = false;
    // Answer Topic
    auto lambda_request_answer = [&sync] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        OrMap sample;
        //std::cout << "Mensajes sin leer " << sub->get_unread_count() << std::endl;
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                    std::cout << " Received Full Graph from " << m_info.sample_identity.writer_guid() << " whith " << sample.m().size() << " elements" << std::endl;
                    graph->join_full_graph(sample);
                    std::cout << "Synchronized." <<std::endl;
                    sync = true;
                }
            }
        }
    };

    dsrpub_request_answer_call = NewMessageFunctor(this, &work, lambda_request_answer);
    dsrsub_request_answer.init(dsrparticipant.getParticipant(), "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName(),dsrpub_request_answer_call);

    std::this_thread::sleep_for(300ms);   // NEEDED ?

    std::cout << " Requesting the complete graph " << std::endl;
    GraphRequest gr;
    gr.from(agent_name);
    dsrpub_graph_request.write(&gr);

    bool timeout = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!sync and !timeout) 
    {
        std::this_thread::sleep_for(500ms);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > TIMEOUT*3;
    }
    eprosima::fastrtps::Domain::removeSubscriber(dsrsub_request_answer.getSubscriber());
    return sync;
}

AworSet CRDTGraph::translateAwCRDTtoIDL(int id, aworset<N, int> &data) 
{
    AworSet delta_crdt;
    for (auto &kv_dots : data.dots().ds) {
        PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second));
    }
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }
    delta_crdt.id(id);
    return delta_crdt;
}

aworset<N, int> CRDTGraph::translateAwIDLtoCRDT(AworSet &data) 
{
    // Context
    dotcontext<int> dotcontext_aux;
    //auto m = static_cast<std::map<int, int>>(data.dk().cbase().cc());
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, N> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = v;
    // Join
    aworset<N, int> aw = aworset<N, int>(data.id());
    aw.setContext(dotcontext_aux);
    aw.dots().set(ds_aux);
    return aw;
}
