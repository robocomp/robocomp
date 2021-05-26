#include <dsr/api/dsr_rt_api.h>
#include <dsr/api/dsr_api.h>

using namespace DSR;

RT_API::RT_API(DSR::DSRGraph *G_)
{
    G = G_;
}

std::optional<Edge> RT_API::get_edge_RT(const Node &n, uint64_t to)
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
            auto r = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(res->second);
            auto t = G->get_attrib_by_name<rt_translation_att>(res->second);
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

std::optional<Mat::RTMat>  RT_API::get_edge_RT_as_rtmat(const Edge &edge, std::uint64_t timestamp)
{
    auto r_o = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge);
    auto t_o =  G->get_attrib_by_name<rt_translation_att>(edge);
    auto head_o = G->get_attrib_by_name<rt_head_index_att>(edge);
    auto tstamps_o = G->get_attrib_by_name<rt_timestamps_att>(edge);
    if (r_o.has_value() and t_o.has_value() and t_o.value().get().size() >= 3 and r_o.value().get().size() >= 3)
    {
        const auto &t = t_o.value().get();
        const auto &r = r_o.value().get();
        if (timestamp == 0)  // return with the first 3 elements of the arrays
        {
            if (head_o.has_value() and tstamps_o.has_value())
            {
                const auto &head = head_o.value();
                return Mat::RTMat(Eigen::Translation3d(t[head], t[head+1], t[head+2]) *
                                  Eigen::AngleAxisd(r[head], Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(r[head+1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(r[head+2], Eigen::Vector3d::UnitZ()));
            }
            else
                return Mat::RTMat(Eigen::Translation3d(t[0], t[1], t[2]) *
                                  Eigen::AngleAxisd(r[0], Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(r[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(r[2], Eigen::Vector3d::UnitZ()));
        }
        else  // timestamp not 0
        {
            if (head_o.has_value() and tstamps_o.has_value())
            {
                const auto &tstamps = tstamps_o.value().get();
                auto i = std::ranges::min_element(tstamps, [timestamp](float x, float y) { return fabs(x - timestamp) < fabs(y - timestamp); });
                auto bix = std::distance(begin(tstamps), i) * BLOCK_SIZE;
                return Mat::RTMat(Eigen::Translation3d(t[bix], t[bix + 1], t[bix + 2]) *
                                  Eigen::AngleAxisd(r[bix], Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(r[bix + 1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(r[bix + 2], Eigen::Vector3d::UnitZ()));
            }
            else
            {
                qWarning() << __FUNCTION__ << "Not head or no timestamps found in RT edge from node "  << edge.from() << " to: " << edge.to() << " Returning first element in array";
                return Mat::RTMat(Eigen::Translation3d(t[0], t[1], t[2]) *
                                  Eigen::AngleAxisd(r[0], Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(r[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(r[2], Eigen::Vector3d::UnitZ()));
            }
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "NO translation or rotation found in RT edge from node " << edge.from() << " to: " << edge.to();
        return {};
    }
}

std::optional<Eigen::Vector3d> RT_API::get_translation(const Node &n, uint64_t to, std::uint64_t timestamp)
{
    if( auto edge = get_edge_RT(n, to); edge.has_value())
    {
        auto t_o = G->get_attrib_by_name<rt_translation_att>(edge.value());
        auto head_o = G->get_attrib_by_name<rt_head_index_att>(edge.value());
        auto tstamps_o = G->get_attrib_by_name<rt_timestamps_att>(edge.value());
        if (t_o.has_value() and t_o.value().get().size() >= 3)
        {
            const auto &t = t_o.value().get();
            if (timestamp == 0)
            {
                if (head_o.has_value() and tstamps_o.has_value())
                    return Eigen::Vector3d(t[head_o.value()], t[head_o.value() + 1], t[head_o.value() + 2]);
                else
                    return Eigen::Vector3d(t[0], t[1], t[2]);
            }
            else  // timestamp not 0
            {
                if (head_o.has_value() and tstamps_o.has_value())
                {
                    // get first index of tstamps whose value is greater than timestamp
                    const auto &tstamps = tstamps_o.value().get();
                    auto i = std::ranges::min_element(tstamps, [timestamp](float x, float y) { return fabs(x - timestamp) < fabs(y - timestamp); });
                    auto bix = std::distance(begin(tstamps), i) * BLOCK_SIZE;
                    return Eigen::Vector3d(t[bix], t[bix + 1], t[bix + 2]);
                }
                else
                {
                    qWarning() << __FUNCTION__ << " Not timestamp or not head found in RT edge from node "  << QString::fromStdString(n.name()) << " to: " << to << " Returning first element in array";
                    return Eigen::Vector3d(t[0], t[1], t[2]);
                }
            }
        }
        else
        {
            qWarning() << __FUNCTION__ << " NO translation found in RT edge from node " << QString::fromStdString(n.name()) << " to: " << to ;
            return {};
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << "NO RT edge found from node " << QString::fromStdString(n.name()) << " to: " << to ;
        return {};
    }
}

std::optional<Eigen::Vector3d> RT_API::get_translation(uint64_t node_id, uint64_t to, std::uint64_t timestamp)
{
    if( const auto node = G->get_node(node_id); node.has_value())
        return get_translation(node.value(), to, timestamp);
    else
        return {};
}

void RT_API::insert_or_assign_edge_RT(Node &n, uint64_t to, const std::vector<float> &trans, const std::vector<float> &rot_euler)
{
    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<std::vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<std::vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(G->_mutex);
        if (G->in(to))
        {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(G->agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(trans)); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(rot_euler)); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rt_rotation_euler_xyz", mvreg<CRDTAttribute> ());
            it->second.write(std::move(rot));
            auto [it2, new_el2] = e.attrs().emplace("rt_translation", mvreg<CRDTAttribute> ());
            it2->second.write(std::move(tr));


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

            if (auto x = G->get_crdt_attrib_by_name<level_att>(to_n.value()); x.has_value())
            {
                if (x.value() != G->get_node_level(n).value() + 1)
                {
                    no_send = !G->modify_attrib_local<level_att>(to_n.value(),  G->get_node_level(n).value() + 1 );
                }
            }
            else
            {
                no_send = !G->add_attrib_local<level_att>(to_n.value(),  G->get_node_level(n).value() + 1 );
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

        if (!no_send and node2.has_value()) G->dsrpub_node_attrs.write(&node2.value());

        emit G->update_edge_attr_signal(n.id(), to,{"rt_rotation_euler_xyz", "rt_translation"});
        emit G->update_edge_signal(n.id(), to, "RT");
        if (!no_send)
        {
            emit G->update_node_signal(to_n->id(), to_n->type());
            emit G->update_node_attr_signal(to_n->id(), {"level", "parent"});
        }
    }
}

void RT_API::insert_or_assign_edge_RT(Node &n, uint64_t to, std::vector<float> &&trans, std::vector<float> &&rot_euler)
{
    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<std::vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<std::vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(G->_mutex);
        if (G->in(to))
        {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(G->agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(std::move(trans))); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(std::move(rot_euler))); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rt_rotation_euler_xyz", mvreg<CRDTAttribute> ());
            it->second.write(std::move(rot));
            auto [it2, new_el2] = e.attrs().emplace("rt_translation", mvreg<CRDTAttribute> ());
            it2->second.write(std::move(tr));


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

            if (auto x = G->get_crdt_attrib_by_name<level_att>(to_n.value()); x.has_value())
            {
                if (x.value() != G->get_node_level(n).value() + 1)
                {
                    no_send = !G->modify_attrib_local<level_att>(to_n.value(),  G->get_node_level(n).value() + 1 );
                }
            }
            else
            {
                no_send = !G->add_attrib_local<level_att>(to_n.value(),  G->get_node_level(n).value() + 1 );
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

        if (!no_send and node2.has_value()) G->dsrpub_node_attrs.write(&node2.value());

        emit G->update_edge_attr_signal(n.id(), to,{"rt_rotation_euler_xyz", "rt_translation"});
        emit G->update_edge_signal(n.id(), to, "RT");
        if (!no_send)
        {
            emit G->update_node_signal(to_n->id(), to_n->type());
            emit G->update_node_attr_signal(to_n->id(), {"level", "parent"});
        }
    }
}
