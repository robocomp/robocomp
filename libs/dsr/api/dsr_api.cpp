//
// Created by crivac on 5/02/19.
//

#include <dsr/api/dsr_api.h>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <utility>
#include <cmath>

#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>


#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>

using namespace DSR;

using namespace std::literals;

/////////////////////////////////////////////////
///// PUBLIC METHODS
/////////////////////////////////////////////////

DSRGraph::DSRGraph(uint64_t root, std::string name, int id, const std::string &dsr_input_file, bool all_same_host)
        : agent_id(id), agent_name(std::move(name)), copy(false), tp(5), same_host(all_same_host), generator(id)
{

    nodes = Nodes();
    utils = std::make_unique<Utilities>(this);
    qDebug() << "Agent name: " << QString::fromStdString(agent_name);

    // RTPS Create participant
    auto[suc, participant_handle] = dsrparticipant.init(agent_id, all_same_host,
                                                        ParticipantChangeFunctor(this, [&](DSR::DSRGraph *graph,
                                                                eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info)
                                                                {
                                                                    if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
                                                                    {
                                                                        std::unique_lock<std::mutex> lck(participant_set_mutex);
                                                                        graph->participant_set.insert({info.info.m_participantName.to_string(), false});
                                                                    }
                                                                    else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
                                                                             info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT)
                                                                    {
                                                                        std::unique_lock<std::mutex> lck(participant_set_mutex);
                                                                        graph->participant_set.erase(info.info.m_participantName.to_string());
                                                                    }
                                                                }));

    // RTPS Initialize publisher with general topic
    auto [res, pub, writer] = dsrpub_node.init(participant_handle, dsrparticipant.getNodeTopic());
    auto [res2, pub2, writer2] = dsrpub_node_attrs.init(participant_handle, dsrparticipant.getAttNodeTopic());

    auto [res3, pub3, writer3] = dsrpub_edge.init(participant_handle, dsrparticipant.getEdgeTopic());
    auto [res4, pub4, writer4] = dsrpub_edge_attrs.init(participant_handle, dsrparticipant.getAttEdgeTopic());

    auto [res5, pub5, writer5] = dsrpub_graph_request.init(participant_handle, dsrparticipant.getGraphRequestTopic());
    auto [res6, pub6, writer6] = dsrpub_request_answer.init(participant_handle, dsrparticipant.getGraphTopic());

    dsrparticipant.add_publisher(dsrparticipant.getNodeTopic()->get_name(), {pub, writer});
    dsrparticipant.add_publisher(dsrparticipant.getAttNodeTopic()->get_name(), {pub2, writer2});
    dsrparticipant.add_publisher(dsrparticipant.getEdgeTopic()->get_name(), {pub3, writer3});
    dsrparticipant.add_publisher(dsrparticipant.getAttEdgeTopic()->get_name(), {pub4, writer4});
    dsrparticipant.add_publisher(dsrparticipant.getGraphRequestTopic()->get_name(), {pub5, writer5});
    dsrparticipant.add_publisher(dsrparticipant.getGraphTopic()->get_name(), {pub6, writer6});

    // RTPS Initialize comms threads
    if (!dsr_input_file.empty())
    {
        try
        {
            read_from_json_file(dsr_input_file);
            qDebug() << __FUNCTION__ << "Warning, graph read from file " << QString::fromStdString(dsr_input_file);
        }
        catch(const DSR::DSRException& e)
        {
            std::cout << e.what() << '\n';
            qFatal("Aborting program. Cannot continue without intial file");
        }
        start_fullgraph_server_thread();
        start_subscription_threads(false);
    }
    else
    {
        start_subscription_threads(false);     // regular subscription to deltas
        auto [response, repeated]  = start_fullgraph_request_thread();    // for agents that want to request the graph for other agent

        if(!response)
        {
            dsrparticipant.remove_participant_and_entities(); // Remove a Participant and all associated publishers and subscribers.

            if (repeated)
            {
                qFatal("%s", (std::string("There is already an agent connected with the id: ") + std::to_string(agent_id)).c_str());
            }
            else {
                qFatal("DSRGraph aborting: could not get DSR from the network after timeout");
            }
        }
    }
    qDebug() << __FUNCTION__ << "Constructor finished OK";
}

DSRGraph::~DSRGraph()
{
    qDebug() << "Removing DSRGraph";
    if (!copy) {
        qDebug() << "Removing rtps participant";
    }
}

//////////////////////////////////////
/// NODE METHODS
/////////////////////////////////////

std::optional<DSR::Node> DSRGraph::get_node(const std::string &name)
{
    std::shared_lock<std::shared_mutex> lock(_mutex);
    if (name.empty()) return {};
    std::optional<uint64_t> id = get_id_from_name(name);
    if (id.has_value())
    {
        std::optional<CRDTNode> n = get_(id.value());
        if (n.has_value()) return Node(n.value());
    }
    return {};
}

std::optional<DSR::Node> DSRGraph::get_node(uint64_t id)
{
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<CRDTNode> n = get_(id);
    if (n.has_value()) return Node(n.value());
    return {};
}


std::tuple<bool, std::optional<IDL::MvregNode>> DSRGraph::insert_node_(const CRDTNode &node)
{
    if (deleted.find(node.id()) == deleted.end())
    {
        if (!nodes[node.id()].empty() and nodes[node.id()].read_reg() == node)
        {
            return {true, {}};
        }

        mvreg<CRDTNode> delta = nodes[node.id()].write(node);
        update_maps_node_insert(node.id(), node);

        return {true, translate_node_mvCRDT_to_IDL(agent_id, node.id(), delta)};
    }
    return {false, {}};
}


std::optional<uint64_t> DSRGraph::insert_node(Node &node)
{
    std::optional<IDL::MvregNode> aw;
    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        std::shared_lock<std::shared_mutex> lck_cache(_mutex_cache_maps);
        uint64_t new_node_id = generator.generate();
        node.id(new_node_id);
        if (node.name().empty() or name_map.find(node.name()) != name_map.end())
            node.name(node.type() + "_" + id_generator::hex_string(new_node_id));
        lck_cache.unlock();
        std::tie(r, aw) = insert_node_(user_node_to_crdt(node));

    }
    if (r)
    {
        if (!copy)
        {
            if (aw.has_value())
            {
                dsrpub_node.write(&aw.value());
                emit update_node_signal(node.id(), node.type());
                for (const auto &[k, v]: node.fano())
                {
                    emit update_edge_signal(node.id(), k.first, k.second);
                }
            }
        }
        return node.id();
    }
    return {};
}

std::tuple<bool, std::optional<std::vector<IDL::MvregNodeAttr>>> DSRGraph::update_node_(const CRDTNode &node)
{

    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].empty()) {

            std::vector<IDL::MvregNodeAttr> atts_deltas;
            auto &iter = nodes[node.id()].read_reg().attrs();
            //New attributes and updates.
            for (auto &[k, att]: node.attrs()) {
                if (iter.find(k) == iter.end()) {
                    mvreg<CRDTAttribute> mv;
                    iter.insert(make_pair(k, mv));
                }
                if (iter[k].empty() or att.read_reg() != iter.at(k).read_reg()) {
                    auto delta = iter[k].write(att.read_reg());
                    atts_deltas.emplace_back(
                            translate_node_attr_mvCRDT_to_IDL(agent_id, node.id(), node.id(), k, delta));

                }
            }
            //Remove old attributes.
            auto it_a = iter.begin();
            while (it_a != iter.end()) {
                const std::string &k = it_a->first;
                const std::string type = it_a->first;
                if (ignored_attributes.find(k) != ignored_attributes.end()) {
                    it_a = iter.erase(it_a);
                } else if (node.attrs().find(k) == node.attrs().end()) {
                    auto delta = iter[k].reset();
                    it_a = iter.erase(it_a);
                    atts_deltas.emplace_back(
                            translate_node_attr_mvCRDT_to_IDL(node.agent_id(), node.id(), node.id(), k, delta));

                } else {
                    it_a++;
                }
            }

            return {true, atts_deltas};
        }
    }

    return {false, {}};
}


bool DSRGraph::update_node(Node &node) {
    bool r = false;
    std::optional<std::vector<IDL::MvregNodeAttr>> vec_node_attr;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        std::shared_lock<std::shared_mutex> lck_cache(_mutex_cache_maps);
        if (deleted.find(node.id()) != deleted.end())
            throw std::runtime_error(
                    (std::string("Cannot update node in G, " + std::to_string(node.id()) + " is deleted") + __FILE__ +
                     " " +
                     __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        else if ((id_map.find(node.id()) != id_map.end() and id_map.at(node.id()) != node.name()) or
                 (name_map.find(node.name()) != name_map.end() and name_map.at(node.name()) != node.id()))
            throw std::runtime_error(
                    (std::string("Cannot update node in G, id and name must be unique") + __FILE__ + " " +
                     __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        else if (nodes.find(node.id()) != nodes.end()) {
            lck_cache.unlock();
            std::tie(r, vec_node_attr) = update_node_(user_node_to_crdt(node));
        }
    }
    if (r) {
        if (!copy) {
            if (vec_node_attr.has_value()) {
                dsrpub_node_attrs.write(&vec_node_attr.value());
                emit update_node_signal(node.id(), node.type());
                std::vector<std::string> atts_names(vec_node_attr->size());
                std::transform(std::make_move_iterator(vec_node_attr->begin()),
                               std::make_move_iterator(vec_node_attr->end()),
                               atts_names.begin(),
                               [](auto &&x) { return x.attr_name(); });
                emit update_node_attr_signal(node.id(), atts_names);

            }
        }
    }
    return r;
}


std::tuple<bool, std::vector<std::tuple<uint64_t, uint64_t, std::string>>, std::optional<IDL::MvregNode>, std::vector<IDL::MvregEdge>>
DSRGraph::delete_node_(uint64_t id) {

    std::vector<std::tuple<uint64_t, uint64_t, std::string>> edges_;
    std::vector<IDL::MvregEdge> aw;

    //1. Get and remove node.
    auto node = get_(id);
    if (!node.has_value()) return make_tuple(false, edges_, std::nullopt, aw);
    for (const auto &v : node.value().fano()) { // Delete all edges from this node.
        edges_.emplace_back(make_tuple(id, v.first.first, v.first.second));
    }
    // Get remove delta.
    auto delta = nodes[id].reset();
    IDL::MvregNode delta_remove = translate_node_mvCRDT_to_IDL(agent_id, id, delta);
    update_maps_node_delete(id, node.value());


    //2. search and remove edges.
    //For each node check if there is an edge to remove.
    for (auto &[k, v] : nodes) {
        std::shared_lock<std::shared_mutex> lck_cache(_mutex_cache_maps);
        if (edges.find({k, id}) == edges.end()) continue;
        // Remove all edges between them
        auto &visited_node = v.read_reg();
        //std::cout << "SIZE: " << visited_node.fano().size() << std::endl;

        for (const auto &key : edges.at({k, id})) {
            //std::cout << "SIZE EDGE: " << visited_node.fano().at({id, key}).dk.ds.size()<< std::endl;

            auto delta_fano = visited_node.fano().at({id, key}).reset();
            aw.emplace_back(translate_edge_mvCRDT_to_IDL(agent_id, k, id, key, delta_fano));
            visited_node.fano().erase({id, key});
            edges_.emplace_back(make_tuple(visited_node.id(), id, key));

        }

        lck_cache.unlock();
        //Remove all from cache
        update_maps_edge_delete(k, id);

    }
    return make_tuple(true, edges_, delta_remove, aw);

}

bool DSRGraph::delete_node(const std::string &name) {

    bool result = false;
    std::vector<std::tuple<uint64_t, uint64_t, std::string>> edges_;
    std::optional<IDL::MvregNode> deleted_node;
    std::vector<IDL::MvregEdge> aw_;

    std::optional<uint64_t> id = {};
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id = get_id_from_name(name);
        if (id.has_value()) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id.value());
        } else {
            return false;
        }
    }

    if (result) {
        if (!copy) {
            emit del_node_signal(id.value());
            dsrpub_node.write(&deleted_node.value());

            for (auto &a  : aw_) {
                dsrpub_edge.write(&a);
            }
            for (auto &[id0, id1, label] : edges_)
                    emit del_edge_signal(id0, id1, label);
        }
        return true;
    }
    return false;

}

bool DSRGraph::delete_node(uint64_t id) {

    bool result;
    std::vector<std::tuple<uint64_t, uint64_t, std::string>> edges_;
    std::optional<IDL::MvregNode> deleted_node;
    std::vector<IDL::MvregEdge> aw_;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(id)) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id);
        } else {
            return false;
        }
    }

    if (result) {
        if (!copy) {

            //std::cout << "[DELETE NODE] : " << id<< std::endl;
            emit del_node_signal(id);
            dsrpub_node.write(&deleted_node.value());

            for (auto &a  : aw_) {
                //std::cout << "  [DELETE EDGE] : " << a.from() << " " << a.to() << " type: " << a.type() << std::endl;
                dsrpub_edge.write(&a);
            }
            for (auto &[id0, id1, label] : edges_)
                    emit del_edge_signal(id0, id1, label);
        }
        return true;
    }

    return false;
}


std::vector<DSR::Node> DSRGraph::get_nodes_by_type(const std::string &type) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::vector<Node> nodes_;
    if (nodeType.find(type) != nodeType.end()) {
        for (auto &id: nodeType[type]) {
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
std::optional<CRDTEdge> DSRGraph::get_edge_(uint64_t from, uint64_t to, const std::string &key) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    if (in(from) && in(to)) {
        auto n = get_(from);
        if (n.has_value()) {
            IDL::EdgeKey ek;
            ek.to(to);
            ek.type(key);
            auto edge = n.value().fano().find({to, key});
            if (edge != n.value().fano().end()) {
                return edge->second.read_reg();
            }
        }
    }
    return {};
}

std::optional<DSR::Edge> DSRGraph::get_edge(const std::string &from, const std::string &to, const std::string &key) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<uint64_t> id_from = get_id_from_name(from);
    std::optional<uint64_t> id_to = get_id_from_name(to);
    if (id_from.has_value() and id_to.has_value()) {
        auto edge_opt = get_edge_(id_from.value(), id_to.value(), key);
        if (edge_opt.has_value()) return Edge(edge_opt.value());
    }
    return {};
}

std::optional<DSR::Edge> DSRGraph::get_edge(uint64_t from, uint64_t to, const std::string &key) {
    auto edge_opt = get_edge_(from, to, key);
    if (edge_opt.has_value()) return Edge(edge_opt.value());
    return {};
}

std::optional<Edge> DSRGraph::get_edge(const Node &n, const std::string &to, const std::string &key) {
    std::optional<uint64_t> id_to = get_id_from_name(to);
    if (id_to.has_value()) {
        return (n.fano().find({id_to.value(), key}) != n.fano().end()) ?
               std::make_optional(n.fano().find({id_to.value(), key})->second) :
               std::nullopt;
    }
    return {};
}

std::optional<Edge> DSRGraph::get_edge(const Node &n, uint64_t to, const std::string &key) {
    IDL::EdgeKey ek;
    ek.to(to);
    ek.type(key);
    return (n.fano().find({to, key}) != n.fano().end()) ?
           std::make_optional(n.fano().find({to, key})->second) :
           std::nullopt;
}

std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>>
DSRGraph::insert_or_assign_edge_(const CRDTEdge &attrs, uint64_t from, uint64_t to) {

    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;


    if (nodes.find(from) != nodes.end()) {
        auto &node = nodes[from].read_reg();
        //check if we are creating an edge or we are updating it.
        if (node.fano().find({to, attrs.type()}) != node.fano().end()) {//Update
            std::vector<IDL::MvregEdgeAttr> atts_deltas;
            auto iter = nodes[from].read_reg().fano().find({attrs.to(), attrs.type()});
            if (iter != nodes[from].read_reg().fano().end()) {
                auto &iter_edge = iter->second.read_reg().attrs();
                for (auto[k, att]: attrs.attrs()) {
                    //comparar igualdad o inexistencia
                    if (iter_edge.find(k) == iter_edge.end()) {
                        mvreg<CRDTAttribute> mv;
                        iter_edge.insert({k, mv});
                    }
                    if (iter_edge[k].empty() or
                        att.read_reg() !=
                        iter_edge.at(k).read_reg()) {
                        auto delta = iter_edge.at(k).write(att.read_reg());//*iter_edge.at(k).read().begin());
                        atts_deltas.emplace_back(
                                translate_edge_attr_mvCRDT_to_IDL(agent_id, from, from, to, attrs.type(), k, delta));

                    }
                }
                auto it = iter_edge.begin();
                while (it != iter_edge.end()) {
                    if (attrs.attrs().find(it->first) == attrs.attrs().end()) {
                        std::string att = it->first;
                        auto delta = iter_edge[it->first].reset();
                        it = iter_edge.erase(it);
                        atts_deltas.emplace_back(
                                translate_edge_attr_mvCRDT_to_IDL(agent_id, from, from, to, attrs.type(), att, delta));

                    } else {
                        it++;
                    }
                }

                return {true, {}, atts_deltas};
            }
        } else { // Insert
            mvreg<CRDTEdge> mv;
            node.fano().insert({{to, attrs.type()}, mv});
            auto delta = node.fano()[{to, attrs.type()}].write(attrs);
            update_maps_edge_insert(from, to, attrs.type());

            return {true, translate_edge_mvCRDT_to_IDL(agent_id, from, to, attrs.type(), delta), {}};
        }
    }
    return {false, {}, {}};
}


bool DSRGraph::insert_or_assign_edge(const Edge &attrs) {
    bool r = false;
    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        uint64_t from = attrs.from();
        uint64_t to = attrs.to();
        if (in(from) && in(to)) {
            std::tie(r, delta_edge, delta_attrs) = insert_or_assign_edge_(user_edge_to_crdt(attrs), from, to);
        } else {
            std::cout << __FUNCTION__ << ":" << __LINE__ << " Error. ID:" << from << " or " << to
                      << " not found. Cant update. " << std::endl;
            return false;
        }
    }
    if (r) {
        if (!copy) {

            emit update_edge_signal(attrs.from(), attrs.to(), attrs.type());

            if (delta_edge.has_value()) { //Insert
                dsrpub_edge.write(&delta_edge.value());
            }
            if (delta_attrs.has_value()) { //Update
                dsrpub_edge_attrs.write(&delta_attrs.value());
                std::vector<std::string> atts_names(delta_attrs->size());
                std::transform(std::make_move_iterator(delta_attrs->begin()),
                               std::make_move_iterator(delta_attrs->end()),
                               atts_names.begin(),
                               [](auto &&x) { return x.attr_name(); });

                emit update_edge_attr_signal(attrs.from(), attrs.to(), atts_names);

            }
        }
    }
    return true;
}


std::optional<IDL::MvregEdge> DSRGraph::delete_edge_(uint64_t from, uint64_t to, const std::string &key) {
    if (nodes.find(from) != nodes.end()) {
        auto &node = nodes.at(from).read_reg();
        if (node.fano().find({to, key}) != node.fano().end()) {
            auto delta = node.fano().at({to, key}).reset();
            node.fano().erase({to, key});
            update_maps_edge_delete(from, to, key);
            return translate_edge_mvCRDT_to_IDL(agent_id, from, to, key, delta);
        }
    }
    return {};
}

bool DSRGraph::delete_edge(uint64_t from, uint64_t to, const std::string &key) {

    std::optional<IDL::MvregEdge> delta;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (!in(from) || !in(to)) return false;
        delta = delete_edge_(from, to, key);

    }
    if (delta.has_value()) {
        if (!copy) {
            emit del_edge_signal(from, to, key);
            dsrpub_edge.write(&delta.value());
        }
        return true;
    }
    return false;

}

bool DSRGraph::delete_edge(const std::string &from, const std::string &to, const std::string &key) {

    std::optional<uint64_t> id_from = {};
    std::optional<uint64_t> id_to = {};
    std::optional<IDL::MvregEdge> delta;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id_from = get_id_from_name(from);
        id_to = get_id_from_name(to);

        if (id_from.has_value() && id_to.has_value()) {
            delta = delete_edge_(id_from.value(), id_to.value(), key);

        }
    }

    if (delta.has_value()) {
        if (!copy) {
            emit del_edge_signal(id_from.value(), id_to.value(), key);
            dsrpub_edge.write(&delta.value());
        }
        return true;
    }
    return false;
}


std::vector<DSR::Edge> DSRGraph::get_node_edges_by_type(const Node &node, const std::string &type) {
    std::vector<Edge> edges_;
    for (auto &[key, edge] : node.fano())
        if (key.second == type)
            edges_.emplace_back(Edge(edge));
    return edges_;
}

std::vector<DSR::Edge> DSRGraph::get_edges_by_type(const std::string &type) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::shared_lock<std::shared_mutex> lock_cache(_mutex_cache_maps);
    std::vector<Edge> edges_;
    if (edgeType.find(type) != edgeType.end()) {
        for (auto &[from, to] : edgeType.at(type)) {
            auto n = get_edge_(from, to, type);
            if (n.has_value())
                edges_.emplace_back(Edge(n.value()));
        }
    }
    return edges_;
}

std::vector<DSR::Edge> DSRGraph::get_edges_to_id(uint64_t id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::shared_lock<std::shared_mutex> lock_cache(_mutex_cache_maps);
    std::vector<Edge> edges_;
    if (to_edges.find(id) != to_edges.end()) {
        for (const auto &[k, v] : to_edges.at(id)) {
            auto n = get_edge_(k, id, v);
            if (n.has_value())
                edges_.emplace_back(Edge(n.value()));
        }
    }

    return edges_;
}

std::optional<std::map<std::pair<uint64_t, std::string>, DSR::Edge>> DSRGraph::get_edges(uint64_t id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<Node> n = get_node(id);
    if (n.has_value()) {
        return n->fano();
    }
    return std::nullopt;

}


/////////////////////////////////////////////////
///// Utils
/////////////////////////////////////////////////

std::map<uint64_t, DSR::Node> DSRGraph::getCopy() const {
    std::map<uint64_t, Node> mymap;
    std::shared_lock<std::shared_mutex> lock(_mutex);

    for (auto &[key, val] : nodes)
        mymap[key] = val.read_reg();

    return mymap;
}

std::vector<uint64_t> DSRGraph::getKeys() const {
    std::vector<uint64_t> keys;
    std::shared_lock<std::shared_mutex> lock(_mutex);

    for (auto &[key, val] : nodes)
        keys.emplace_back(key);

    return keys;
}


//////////////////////////////////////////////////////////////////////////////
/////  CORE
//////////////////////////////////////////////////////////////////////////////

std::optional<CRDTNode> DSRGraph::get(uint64_t id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    return get_(id);
}

std::optional<CRDTNode> DSRGraph::get_(uint64_t id) {

    if (in(id)) {
        if (!nodes[id].empty()) {
            return std::make_optional(nodes[id].read_reg());
        }
    }
    return {};
}

std::optional<std::int32_t> DSRGraph::get_node_level(const Node &n) {
    return get_attrib_by_name<level_att>(n);
}

std::optional<uint64_t> DSRGraph::get_parent_id(const Node &n) {
    return get_attrib_by_name<parent_att>(n);
}

std::optional<DSR::Node> DSRGraph::get_parent_node(const Node &n) {
    auto p = get_attrib_by_name<parent_att>(n);
    if (p.has_value()) {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        auto tmp = get_(p.value());
        if (tmp.has_value()) return Node(tmp.value());
    }
    return {};
}


std::string DSRGraph::get_node_type(Node &n) {
    return n.type();
}

//////////////////////////////////////////////////////////////////////////////////////////////7

inline void DSRGraph::update_maps_node_delete(uint64_t id, const std::optional<CRDTNode> &n) {
    nodes.erase(id);

    std::unique_lock<std::shared_mutex> lck(_mutex_cache_maps);
    if (id_map.find(id) != id_map.end()){
        name_map.erase(id_map.at(id));
        id_map.erase(id);
    }
    deleted.insert(id);
    to_edges.erase(id);

    if (n.has_value()) {

        if (nodeType.find(n->type()) != nodeType.end()) {
            nodeType.at(n->type()).erase(id);
            if (nodeType.at(n->type()).empty()) nodeType.erase(n->type());
        }

        for (const auto &[k, v] : n->fano()) {
            if (auto tuple = std::pair{id, v.read_reg().to()}; edges.find(tuple) != edges.end()) {
                edges.at(tuple).erase(k.second);
                if (edges.at(tuple).empty()) edges.erase(tuple);
            }
            if (auto tuple = std::pair{id, k.first}; edgeType.find(k.second) != edgeType.end()) {
                edgeType.at(k.second).erase(tuple);
                if (edgeType.at(k.second).empty())edgeType.erase(k.second);
            }
            if (auto tuple = std::pair{id, k.second}; to_edges.find(k.first) != to_edges.end()) {
                to_edges.at(k.first).erase(tuple);
                if (to_edges.at(k.first).empty()) to_edges.erase(k.first);
            }
        }
    }
}

inline void DSRGraph::update_maps_node_insert(uint64_t id, const CRDTNode &n) {

    std::unique_lock<std::shared_mutex> lck(_mutex_cache_maps);

    name_map[n.name()] = id;
    id_map[id] = n.name();
    nodeType[n.type()].emplace(id);

    for (const auto &[k, v] : n.fano()) {
        edges[{id, k.first}].insert(k.second);
        edgeType[k.second].insert({id, k.first});
        to_edges[k.first].insert({id, k.second});

    }
}


inline void DSRGraph::update_maps_edge_delete(uint64_t from, uint64_t to, const std::string &key) {

    std::unique_lock<std::shared_mutex> lck(_mutex_cache_maps);

    //if key is empty we delete all edges to the node from
    if (key.empty()) {
        edges.erase({from, to});

        if (edgeType.find(key) != edgeType.end()) {
            edgeType.at(key).erase({from, to});
            if (edgeType.at(key).empty()) edgeType.erase(key);
        }

        if (to_edges.find(to) != to_edges.end()) {
            auto &set = to_edges.at(to);
            auto it = set.begin();
            while (it != set.end()){
                if (it->first == from)
                    it = set.erase(it);
                else
                    it++;
            }
            if (set.empty()) to_edges.erase(to);
        }

    } else {
        if (auto tuple = std::pair{from, to}; edges.find(tuple) != edges.end()) {
            edges.at(tuple).erase(key);
            edges.erase({from, to});
        }

        if (to_edges.find(to) != to_edges.end()) {
            to_edges.at(to).erase({from, key});
            if (to_edges.at(to).empty()) to_edges.erase(to);
        }

        if (edgeType.find(key) != edgeType.end()) {
            edgeType.at(key).erase({from, to});
            if (edgeType.at(key).empty()) edgeType.erase(key);
        }

    }
}

inline void DSRGraph::update_maps_edge_insert(uint64_t from, uint64_t to, const std::string &key) {
    std::unique_lock<std::shared_mutex> lck(_mutex_cache_maps);

    edges[{from, to}].insert(key);
    to_edges[to].insert({from, key});
    edgeType[key].insert({from, to});

}


std::optional<uint64_t> DSRGraph::get_id_from_name(const std::string &name) {
    std::shared_lock<std::shared_mutex> lck(_mutex_cache_maps);
    auto v = name_map.find(name);
    if (v != name_map.end()) return v->second;
    return {};
}

std::optional<std::string> DSRGraph::get_name_from_id(uint64_t id) {
    std::shared_lock<std::shared_mutex> lck(_mutex_cache_maps);
    auto v = id_map.find(id);
    if (v != id_map.end()) return v->second;
    return {};
}

size_t DSRGraph::size() {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    return nodes.size();
}

bool DSRGraph::in(uint64_t id) const {
    return nodes.find(id) != nodes.end();
}

bool DSRGraph::empty(const uint64_t &id) {
    if (nodes.find(id) != nodes.end()) {
        return nodes.at(id).empty();
    } else
        return false;
}

void DSRGraph::join_delta_node(IDL::MvregNode &&mvreg) {

    std::optional<CRDTNode> maybe_deleted_node = {};
    try {

        bool signal = false, ok = false;
        auto id = mvreg.id();
        auto d = translate_node_mvIDL_to_CRDT(std::move(mvreg));
        bool d_empty = d.empty();

        std::optional<std::unordered_set<std::pair<uint64_t, std::string>,hash_tuple>> cache_map_to_edges = {};
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            if (deleted.find(id) == deleted.end()) {
                ok = true;
                nodes[id].join(std::move(d));
                if (nodes[id].empty() or d_empty) {
                    nodes.erase(id);
                    maybe_deleted_node = (nodes[id].empty()) ?
                         std::nullopt : std::make_optional(nodes[id].read_reg());
                    cache_map_to_edges = to_edges[id];
                    //Update Maps
                    update_maps_node_delete(id, maybe_deleted_node);
                } else {

                    signal = true;
                    update_maps_node_insert(id, nodes[id].read_reg());
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_node_signal(id, nodes[id].read_reg().type());
                for (auto &[k, v] : nodes[id].read_reg().fano()) {
                    emit update_edge_signal(id, k.first, k.second);
                }
            } else {
                emit del_node_signal(id);
                if (maybe_deleted_node.has_value()) {
                    for (auto &node: maybe_deleted_node->fano()) {
                        emit del_edge_signal(node.second.read_reg().from(), node.second.read_reg().to(),
                                             node.second.read_reg().type());
                    }
                }

                for (const auto &[from, type] : cache_map_to_edges.value()) {
                    emit del_edge_signal(from, id, type);
                }

            }
        }

    }
    catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    }

}


void DSRGraph::join_delta_edge(IDL::MvregEdge &&mvreg) {
    try {
        bool signal = false, ok = false;
        auto from = mvreg.id();
        auto to = mvreg.to();
        std::string type = mvreg.type();

        auto d = translate_edge_mvIDL_to_CRDT(std::move(mvreg));
        bool d_empty = d.empty();
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);

            //Check if the node where we are joining the edge exist.
            if (nodes.find(from) != nodes.end()) {
                ok = true;
                auto &n = nodes[from].read_reg();

                n.fano()[{to, type}].join(std::move(d));

                //Check if we are inserting or deleting.
                if (d_empty or n.fano().find({to, type}) == n.fano().end()) { //Remove
                    n.fano().erase({to, type});
                    //Update maps

                    update_maps_edge_delete(from, to, type);
                } else { //Insert

                    signal = true;
                    //Update maps
                    update_maps_edge_insert(from, to, type);
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_edge_signal(from, to, type);
            } else {
                emit del_edge_signal(from, to, type);
            }
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    }


}


void DSRGraph::join_delta_node_attr(IDL::MvregNodeAttr &&mvreg) {

    try {
        bool ok = false;
        auto id = mvreg.id();
        std::string att_name = mvreg.attr_name();

        auto d = translate_node_attr_mvIDL_to_CRDT(std::move(mvreg));
        bool d_empty = d.empty();
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (deleted.find(id) == deleted.end()
                and nodes.find(id) != nodes.end()) {
                ok = true;
                auto &n = nodes[id].read_reg();
                if (n.attrs().find(att_name) == n.attrs().end()) {
                    ::mvreg<CRDTAttribute> new_mv;
                    n.attrs().insert({att_name, new_mv});
                }

                // std::cout << "JOINING NODE ATTRIBUTE: " << att_name << std::endl;
                n.attrs()[att_name].join(std::move(d));

                //Check if we are inserting or deleting.
                if (d_empty or n.attrs().find(att_name) == n.attrs().end()) { //Remove
                    n.attrs().erase(att_name);
                }


            }
        }

        if (ok) {
            emit update_node_signal(id, nodes[id].read_reg().type());
            emit update_node_attr_signal(id, {att_name});
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    }


}


void DSRGraph::join_delta_edge_attr(IDL::MvregEdgeAttr &&mvreg) {
    try {
        bool ok = false;
        auto from = mvreg.id();
        auto to = mvreg.to();
        std::string type = mvreg.type();
        std::string att_name = mvreg.attr_name();

        auto d = translate_edge_attr_mvIDL_to_CRDT(std::move(mvreg));
        bool d_empty = d.empty();
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (nodes.find(from) != nodes.end()
                and !(nodes[from].read_reg().fano().find({to, type}) == nodes[from].read_reg().fano().end())) {
                ok = true;
                auto &n = nodes[from].read_reg().fano()[{to, type}].read_reg();

                n.attrs()[att_name].join(std::move(d));

                //Check if we are inserting or deleting.
                if (d_empty or n.attrs().find(att_name) == n.attrs().end()) { //Remove
                    n.attrs().erase(att_name);
                }
            }
        }

        if (ok) {
            emit update_edge_signal(from, to, type);
            emit update_edge_attr_signal(from, to, {att_name});
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    }


}

void DSRGraph::join_full_graph(IDL::OrMap &&full_graph) {

    std::vector<std::tuple<bool, uint64_t, std::string, std::optional<CRDTNode>>> updates;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);

        for (auto &[k, val] : full_graph.m()) {
            auto mv = translate_node_mvIDL_to_CRDT(std::move(val));
            bool mv_empty = mv.empty();
            std::optional<CRDTNode> nd = (nodes[k].empty()) ? std::nullopt : std::make_optional(nodes[k].read_reg());

            if (deleted.find(k) == deleted.end()) {
                nodes[k].join(std::move(mv));
                if (mv_empty or nodes[k].empty()) {
                    update_maps_node_delete(k, nd);
                    updates.emplace_back(make_tuple(false, k, "", std::nullopt));
                } else {
                    update_maps_node_insert(k, nodes[k].read_reg());
                    updates.emplace_back(make_tuple(true, k, nodes[k].read_reg().type(), nd));
                }
            }
        }



    }
    for (auto &[signal, id, type, nd] : updates)
        if (signal) {
            //check what change is joined
            if (!nd.has_value() || nd->attrs() != nodes[id].read_reg().attrs()) {
                emit update_node_signal(id, nodes[id].read_reg().type());
            } else if (nd.value() != nodes[id].read_reg()) {
                auto iter = nodes[id].read_reg().fano();
                for (const auto &[k, v] : nd->fano()) {
                    if (iter.find(k) == iter.end())
                            emit del_edge_signal(id, k.first, k.second);
                }
                for (const auto &[k, v] : iter) {
                    if (auto it = nd->fano().find(k); it == nd->fano().end() or it->second != v)
                            emit update_edge_signal(id, k.first, k.second);
                }
            }
        } else {
            emit del_node_signal(id);
        }

}

std::pair<bool, bool> DSRGraph::start_fullgraph_request_thread() {
    return fullgraph_request_thread();
}

void DSRGraph::start_fullgraph_server_thread() {
    auto fullgraph_thread = std::thread(&DSRGraph::fullgraph_server_thread, this);
    if (fullgraph_thread.joinable()) fullgraph_thread.join();
}

void DSRGraph::start_subscription_threads(bool showReceived) {
    auto delta_node_thread = std::thread(&DSRGraph::node_subscription_thread, this, showReceived);
    auto delta_edge_thread = std::thread(&DSRGraph::edge_subscription_thread, this, showReceived);
    auto delta_node_attrs_thread = std::thread(&DSRGraph::node_attrs_subscription_thread, this, showReceived);
    auto delta_edge_attrs_thread = std::thread(&DSRGraph::edge_attrs_subscription_thread, this, showReceived);

    if (delta_node_thread.joinable()) delta_node_thread.join();
    if (delta_edge_thread.joinable()) delta_edge_thread.join();
    if (delta_node_attrs_thread.joinable()) delta_node_attrs_thread.join();
    if (delta_edge_attrs_thread.joinable()) delta_edge_attrs_thread.join();
}

std::map<uint64_t , IDL::MvregNode> DSRGraph::Map() {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::map<uint64_t, IDL::MvregNode> m;
    for (auto kv : nodes) {
        m[kv.first] = translate_node_mvCRDT_to_IDL(agent_id, kv.first, kv.second);
    }
    return m;
}

void DSRGraph::node_subscription_thread(bool showReceived) {
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name, showReceived = showReceived](eprosima::fastdds::dds::DataReader *reader,
                                                 DSR::DSRGraph *graph) {

        try {
            while (true)
            {
                eprosima::fastdds::dds::SampleInfo m_info;
                IDL::MvregNode sample;
                if (reader->take_next_sample(&sample, &m_info) == ReturnCode_t::RETCODE_OK) {
                    if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived) {
                                qDebug() << name << " Received:" << std::to_string(sample.id()).c_str() << " node from: "
                                        << m_info.sample_identity.writer_guid().entityId.value;
                            }
                            tp.spawn_task(&DSRGraph::join_delta_node, this, std::move(sample));
                        }
                    }
                } else {
                    break;
                }
            }
        }
        catch (const std::exception &ex) { std::cerr << ex.what() << std::endl; }

    };
    dsrpub_call_node = NewMessageFunctor(this, lambda_general_topic);
    auto [res, sub, reader] = dsrsub_node.init(dsrparticipant.getParticipant(), dsrparticipant.getNodeTopic(), dsrpub_call_node, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getNodeTopic()->get_name(), {sub, reader});
}

void DSRGraph::edge_subscription_thread(bool showReceived) {
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name, showReceived = showReceived](eprosima::fastdds::dds::DataReader *reader,
                                                 DSR::DSRGraph *graph) {
        try {
            while (true)
            {
                eprosima::fastdds::dds::SampleInfo m_info;
                IDL::MvregEdge sample;
                if (reader->take_next_sample(&sample, &m_info) == ReturnCode_t::RETCODE_OK) {
                    if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived) {
                                qDebug() << name << " Received:" << std::to_string(sample.id()).c_str() << " node from: "
                                        << m_info.sample_identity.writer_guid().entityId.value;
                            }
                            tp.spawn_task(&DSRGraph::join_delta_edge, this, std::move(sample));
                        }
                    }
                } else {
                    break;
                }
            }
        }
        catch (const std::exception &ex) { std::cerr << ex.what() << std::endl; }
    };
    dsrpub_call_edge = NewMessageFunctor(this, lambda_general_topic);
    auto [res, sub, reader]  = dsrsub_edge.init(dsrparticipant.getParticipant(), dsrparticipant.getEdgeTopic(), dsrpub_call_edge, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getEdgeTopic()->get_name(), {sub, reader});

}

void DSRGraph::edge_attrs_subscription_thread(bool showReceived) {
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name, showReceived = showReceived](eprosima::fastdds::dds::DataReader *reader,
                                                 DSR::DSRGraph *graph) {

        try {
            while (true)
            {
                eprosima::fastdds::dds::SampleInfo m_info;
                IDL::MvregEdgeAttrVec samples;
                if (reader->take_next_sample(&samples, &m_info) == ReturnCode_t::RETCODE_OK) {
                    if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                        if (showReceived) {
                            qDebug() << name << " Received:" << samples.vec().size() << " edge attr from: "
                                    << m_info.sample_identity.writer_guid().entityId.value;
                        }
                        for (auto &&sample: samples.vec()) {
                            if (sample.agent_id() != agent_id
                                and graph->ignored_attributes.find(sample.attr_name().data()) == ignored_attributes.end()) {
                                //PRINT_TIME("edge", sample);
                                tp.spawn_task(&DSRGraph::join_delta_edge_attr, this, std::move(sample));
                            }
                        }
                    }
                } else {
                    break;
                }
            }
        }
        catch (const std::exception &ex) { std::cerr << ex.what() << std::endl; }

    };
    dsrpub_call_edge_attrs = NewMessageFunctor(this, lambda_general_topic);
    auto [res, sub, reader] = dsrsub_edge_attrs.init(dsrparticipant.getParticipant(), dsrparticipant.getAttEdgeTopic(),
                           dsrpub_call_edge_attrs, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getAttEdgeTopic()->get_name(), {sub, reader});

    //dsrsub_edge_attrs_stream.init(dsrparticipant.getParticipant(), "DSR_EDGE_ATTRS_STREAM", dsrparticipant.getEdgeAttrTopicName(),
    //                       dsrpub_call_edge_attrs, true);
}

void DSRGraph::node_attrs_subscription_thread(bool showReceived) {
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name, showReceived = showReceived](eprosima::fastdds::dds::DataReader *reader,
                                                 DSR::DSRGraph *graph) {

        try {
            while (true)
            {
                eprosima::fastdds::dds::SampleInfo m_info;
                IDL::MvregNodeAttrVec samples;
                if (reader->take_next_sample(&samples, &m_info) == ReturnCode_t::RETCODE_OK) {
                    if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                        if (showReceived) {
                            qDebug() << name << " Received:" << samples.vec().size() << " node attrs from: "
                                    << m_info.sample_identity.writer_guid().entityId.value;
                        }

                        for (auto &&s: samples.vec()) {
                            if (s.agent_id() != agent_id and
                                graph->ignored_attributes.find(s.attr_name().data()) == ignored_attributes.end()) {
                                tp.spawn_task(&DSRGraph::join_delta_node_attr, this, std::move(s));
                            }
                        }
                    }
                } else {
                    break;
                }
            }
        }
        catch (const std::exception &ex) { std::cerr << ex.what() << std::endl; }

    };
    dsrpub_call_node_attrs = NewMessageFunctor(this, lambda_general_topic);
    auto [res, sub, reader] = dsrsub_node_attrs.init(dsrparticipant.getParticipant(), dsrparticipant.getAttNodeTopic(),
                           dsrpub_call_node_attrs, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getAttNodeTopic()->get_name(), {sub, reader});

}

void DSRGraph::fullgraph_server_thread() {
    auto lambda_graph_request = [&](eprosima::fastdds::dds::DataReader *reader, DSR::DSRGraph *graph) {

        while (true)
        {
            eprosima::fastdds::dds::SampleInfo m_info;
            IDL::GraphRequest sample;
            if (reader->take_next_sample(&sample, &m_info) == ReturnCode_t::RETCODE_OK) {
                if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                    {
                        std::unique_lock<std::mutex> lck(participant_set_mutex);
                        if (auto it = participant_set.find(("Participant_" + std::to_string(sample.id())));
                            it != participant_set.end() and it->second)
                        {
                            lck.unlock();
                            IDL::OrMap mp; mp.id(-1);
                            dsrpub_request_answer.write(&mp);
                            continue;
                        } else {
                            it->second = true;
                            lck.unlock();
                        }
                    }
                    if (static_cast<uint32_t>(sample.id()) != agent_id ) {

                        qDebug() << " Received Full Graph request: from "
                                << m_info.sample_identity.writer_guid().entityId.value;
                        IDL::OrMap mp;
                        mp.id(graph->get_agent_id());
                        mp.m(graph->Map());
                        dsrpub_request_answer.write(&mp);

                        qDebug() << "Full graph written";

                    }
                }
            } else {
                break;
            }
        }
    };
    dsrpub_graph_request_call = NewMessageFunctor(this, lambda_graph_request);
    auto [res, sub, reader] = dsrsub_graph_request.init(dsrparticipant.getParticipant(), dsrparticipant.getGraphRequestTopic(),
                              dsrpub_graph_request_call, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getGraphRequestTopic()->get_name(), {sub, reader});

}

std::pair<bool, bool> DSRGraph::fullgraph_request_thread() {
    bool sync = false;
    bool repeated = false;
    auto lambda_request_answer = [&](eprosima::fastdds::dds::DataReader *reader, DSR::DSRGraph *graph) {

        while (true)
        {
            eprosima::fastdds::dds::SampleInfo m_info;
            IDL::OrMap sample;
            if (reader->take_next_sample(&sample, &m_info) == ReturnCode_t::RETCODE_OK) {
                if (m_info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) {
                    if (sample.id() != graph->get_agent_id()) {
                        if (sample.id() != static_cast<uint32_t>(-1)) {
                            qDebug() << " Received Full Graph from " << m_info.sample_identity.writer_guid().entityId.value
                                    << " whith "
                                    << sample.m().size() << " elements";
                            tp.spawn_task(&DSRGraph::join_full_graph, this, std::move(sample));
                            qDebug() << "Synchronized.";
                            sync = true;
                            break;
                        }
                        else
                        {
                            repeated = true;
                        }
                    }
                }
            } else {
                break;
            }
        }
    };

    dsrpub_request_answer_call = NewMessageFunctor(this, lambda_request_answer);
    auto [res, sub, reader] = dsrsub_request_answer.init(dsrparticipant.getParticipant(), dsrparticipant.getGraphTopic(),
                               dsrpub_request_answer_call, mtx_entity_creation);
    dsrparticipant.add_subscriber(dsrparticipant.getGraphTopic()->get_name(), {sub, reader});

    std::this_thread::sleep_for(300ms);   // NEEDED ?

    qDebug() << " Requesting the complete graph ";

    IDL::GraphRequest gr;
    gr.from(std::to_string(agent_id));
    gr.id(agent_id);
    dsrpub_graph_request.write(&gr);


    bool timeout = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!sync and !timeout and !repeated) {
        std::this_thread::sleep_for(1000ms);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > TIMEOUT * 3;
        qInfo() << " Waiting for the graph ... seconds to timeout ["
                << std::ceil(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 10) / 100.0
                << "/" << TIMEOUT / 1000 * 3 << "] ";
        dsrpub_graph_request.write(&gr);
    }

    dsrparticipant.delete_publisher(dsrparticipant.getGraphRequestTopic()->get_name());
    dsrparticipant.delete_subscriber(dsrparticipant.getGraphTopic()->get_name());

    return { sync, repeated };
}


//////////////////////////////////////////////////
///// PRIVATE COPY
/////////////////////////////////////////////////

DSRGraph::DSRGraph(const DSRGraph &G) : agent_id(G.agent_id), copy(true), tp(1), generator(G.agent_id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::shared_lock<std::shared_mutex> lock_cache(_mutex_cache_maps);
    nodes = G.nodes;
    utils = std::make_unique<Utilities>(this);
    id_map = G.id_map;
    deleted = G.deleted;
    name_map = G.name_map;
    edges = G.edges;
    edgeType = G.edgeType;
    nodeType = G.nodeType;
    same_host = G.same_host;
}

std::unique_ptr<DSRGraph> DSRGraph::G_copy() {
    return std::unique_ptr<DSRGraph>(new DSRGraph(*this));
}

bool DSRGraph::is_copy() const {
    return copy;
}
