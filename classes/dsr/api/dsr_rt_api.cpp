#include "dsr_rt_api.h"
#include "dsr_api.h"

using namespace DSR;

RT_API::RT_API(DSR::DSRGraph *G_)
{
    G = G_;
}

std::optional<Edge> RT_API::get_edge_RT(const Node &n, uint32_t to)
{
    auto edges_ = n.fano();
    auto res = edges_.find({to, "RT"});
    if (res != edges_.end())
        return res->second;
    else
        return {};
}

std::optional<Mat::RTMat> RT_API::get_RT_pose_from_parent(const Node &n)
{
    auto p = G->get_parent_node(n);
    if (p.has_value())
    {
        auto edges_ = p->fano();
        auto res = edges_.find({n.id(),"RT"});
        if (res != edges_.end())
        {
            auto r = G->get_attrib_by_name<rotation_euler_xyz>(res->second);
            auto t = G->get_attrib_by_name<translation>(res->second);
            if (r.has_value() && t.has_value() )
            {
                Mat::RTMat rt(Eigen::Translation3d(t->get()[0], t->get()[1], t->get()[2]) *
                              Eigen::AngleAxisd(r->get()[0], Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(r->get()[1], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(r->get()[2], Eigen::Vector3d::UnitZ()));
                return rt;
            }
        }
    }
    return {};
}

std::optional<Mat::RTMat>  RT_API::get_edge_RT_as_rtmat(const Edge &edge)
{
    auto r = G->get_attrib_by_name<rotation_euler_xyz>(edge);
    auto t =  G->get_attrib_by_name<translation>(edge);
    if (r.has_value() and t.has_value())
    {
        Mat::RTMat rt(Eigen::Translation3d(t->get()[0], t->get()[1], t->get()[2]) *
                      Eigen::AngleAxisd(r->get()[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(r->get()[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(r->get()[2], Eigen::Vector3d::UnitZ()));
        return rt;
    }
    else
        return {};
}

void RT_API::insert_or_assign_edge_RT(Node &n, uint32_t to, const std::vector<float> &trans, const std::vector<float> &rot_euler)
{
    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(G->_mutex);
        if (G->in(to))
        {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(G->agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(trans)); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(rot_euler)); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rotation_euler_xyz", mvreg<CRDTAttribute, uint32_t> ());
            it->second.write(rot);
            auto [it2, new_el2] = e.attrs().emplace("translation", mvreg<CRDTAttribute, uint32_t> ());
            it2->second.write(tr);


            to_n = G->get_(to).value();
            if (auto x = G->get_crdt_attrib_by_name<parent_att>(to_n.value()); x.has_value())
            {
                if ( x.value() != n.id())
                {
                    no_send = !G->modify_attrib_local<parent_att>(to_n.value(), n.id());
                }
            }
            else
            {
                no_send = !G->add_attrib_local<parent_att>(to_n.value(), n.id());
            }

            if (auto x = G->get_crdt_attrib_by_name<level>(to_n.value()); x.has_value())
            {
                if (x.value() != G->get_node_level(n).value() + 1)
                {
                    no_send = !G->modify_attrib_local<level>(to_n.value(),  G->get_node_level(n).value() + 1 );
                }
            }
            else
            {
                no_send = G->add_attrib_local<level>(to_n.value(),  G->get_node_level(n).value() + 1 );
            }

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end())
            {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, node1_insert, std::ignore) = G->insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = G->update_node_(to_n.value());

            }
            else
            {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, std::ignore, node1_update) = G->insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = G->update_node_(to_n.value());

            }
            if (!r1)
            {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
            if (!r2 and !no_send)
            {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(to_n->id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
        } else
            throw std::runtime_error(
                    "Destination node " + std::to_string(to) + " not found in G in insert_or_assign_edge_RT() " +
                    __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (!G->copy)
    {
        if (node1_insert.has_value())
        {
            G->dsrpub_edge.write(&node1_insert.value());
        }
        if (node1_update.has_value()) G->dsrpub_edge_attrs.write(&node1_update.value());
            /*for (auto &d : node1_update.value())
                G->dsrpub_edge_attrs.write(&d);*/
        if (!no_send and node2.has_value()) G->dsrpub_node_attrs.write(&node2.value());
            /*for (auto &d : node2.value())
                G->dsrpub_node_attrs.write(&d);*/

        emit G->update_edge_signal(n.id(), to, "RT");
        if (!no_send) emit G->update_node_signal(to_n->id(), to_n->type());
    }
}

void RT_API::insert_or_assign_edge_RT(Node &n, uint32_t to, std::vector<float> &&trans, std::vector<float> &&rot_euler)
{
    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(G->_mutex);
        if (G->in(to))
        {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(G->agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(std::move(trans))); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(std::move(rot_euler))); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rotation_euler_xyz", mvreg<CRDTAttribute, uint32_t> ());
            it->second.write(rot);
            auto [it2, new_el2] = e.attrs().emplace("translation", mvreg<CRDTAttribute, uint32_t> ());
            it2->second.write(tr);


            to_n = G->get_(to).value();
            if (auto x = G->get_crdt_attrib_by_name<parent_att>(to_n.value()); x.has_value())
            {
                if ( x.value() != n.id())
                {
                    no_send = !G->modify_attrib_local<parent_att>(to_n.value(), n.id());
                }
            }
            else
            {
                no_send = !G->add_attrib_local<parent_att>(to_n.value(), n.id());
            }

            if (auto x = G->get_crdt_attrib_by_name<level>(to_n.value()); x.has_value())
            {
                if (x.value() != G->get_node_level(n).value() + 1)
                {
                    no_send = !G->modify_attrib_local<level>(to_n.value(),  G->get_node_level(n).value() + 1 );
                }
            }
            else
            {
                no_send = G->add_attrib_local<level>(to_n.value(),  G->get_node_level(n).value() + 1 );
            }

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end())
            {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, node1_insert, std::ignore) = G->insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = G->update_node_(to_n.value());

            }
            else
            {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, std::ignore, node1_update) = G->insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = G->update_node_(to_n.value());

            }
            if (!r1)
            {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
            if (!r2 and !no_send)
            {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(to_n->id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
        } else
            throw std::runtime_error(
                    "Destination node " + std::to_string(to) + " not found in G in insert_or_assign_edge_RT() " +
                    __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (!G->copy)
    {
        if (node1_insert.has_value())
        {
            G->dsrpub_edge.write(&node1_insert.value());
        }
        if (node1_update.has_value()) G->dsrpub_edge_attrs.write(&node1_update.value());
        /*for (auto &d : node1_update.value())
            G->dsrpub_edge_attrs.write(&d);*/
        if (!no_send and node2.has_value()) G->dsrpub_node_attrs.write(&node2.value());
        /*for (auto &d : node2.value())
            G->dsrpub_node_attrs.write(&d);*/


        emit G->update_edge_signal(n.id(), to, "RT");
        if (!no_send) emit G->update_node_signal(to_n->id(), to_n->type());
    }
}
