
#include "dsr_inner_api.h"
#include "dsr_api.h"

using namespace DSR;

//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
InnerAPI::InnerAPI(DSR::DSRGraph *G_)
{
    G = G_;
    rt = G->get_rt_api();
    //update signals
    connect(G, &DSR::DSRGraph::update_edge_signal, this, &InnerAPI::add_or_assign_edge_slot);
    connect(G, &DSR::DSRGraph::del_edge_signal, this, &InnerAPI::del_edge_slot);
    connect(G, &DSR::DSRGraph::del_node_signal, this, &InnerAPI::del_node_slot);
}

/// Computation of resultant RTMat going from A to common ancestor and from common ancestor to B (inverted)
std::optional<InnerAPI::Lists> InnerAPI::set_lists(const std::string &destId, const std::string &origId)
{
    std::list<node_matrix> listA, listB;

  	auto an = G->get_node(origId);
	auto bn = G->get_node(destId);
    if ( not an.has_value() or  not bn.has_value())
		return {};
	auto a = an.value(); auto b = bn.value();

	int minLevel = std::min(G->get_node_level(a).value_or(-1), G->get_node_level(b).value_or(-1));
	if (minLevel == -1)
		return {};
	while (G->get_node_level(a).value_or(-1) >= minLevel)
	{
        //qDebug() << "listaA" << a.id() << G->get_node_level(a).value() << G->get_parent_id(a).value();
		auto p_node = G->get_parent_node(a);
      	if( not p_node.has_value())
			break;
		auto edge_rt = rt->get_edge_RT(p_node.value(), a.id()).value();
		auto rtmat = rt->get_edge_RT_as_RTMat(edge_rt).value();
		listA.emplace_back(std::make_tuple(p_node.value().id(), std::move(rtmat)));   // the downwards RT link from parent to a
        a = p_node.value();
	}
	while (G->get_node_level(b).value_or(-1) >= minLevel)
	{
        //qDebug() << "listaB" << b.id() << G->get_node_level(b).value() << G->get_parent_id(b).value();
		auto p_node = G->get_parent_node(b);
		if(not p_node.has_value())
			break;
		auto edge_rt = rt->get_edge_RT(p_node.value(), b.id()).value();
		auto rtmat = rt->get_edge_RT_as_rtmat(edge_rt).value();
        listB.emplace_front(std::make_tuple(p_node.value().id(), std::move(rtmat)));
		b = p_node.value();
	}
	while (a.id() != b.id())
	{
		auto p = G->get_node(G->get_parent_id(a).value_or(-1));
		auto q = G->get_node(G->get_parent_id(b).value_or(-1));
		if(p.has_value() and q.has_value())
		{
            //qDebug() << "listas A&B" << p.value().id() << q.value().id();
	  		listA.push_back(std::make_tuple(p.value().id(), rt->get_edge_RT_as_rtmat(rt->get_edge_RT(p.value(), a.id()).value()).value()));
	  		listB.push_front(std::make_tuple(q.value().id(), rt->get_edge_RT_as_rtmat(rt->get_edge_RT(q.value(), b.id()).value()).value()));
			a = p.value();
			b = q.value();
		}
		else
			return {};
	}
    return std::make_tuple(listA, listB);
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRANSFORMATION MATRIX
////////////////////////////////////////////////////////////////////////////////////////

std::optional<RTMat> InnerAPI::get_transformation_matrix(const std::string &dest, const std::string &orig)
{
    RTMat ret;
    key_transform key = std::make_tuple(dest, orig);
    transform_cache::iterator it = cache.find(key);
    if (it != cache.end())
        ret = it->second;
    else
    {
        auto lists = set_lists(dest, orig);
        if(!lists.has_value())
            return {};
        auto &[listA, listB] = lists.value();

        for(auto &[id, mat]: listA )
        {
            ret = mat*ret;
            node_map[id].push_back(key); // update node cache reference
        }
        for(auto &[id, mat]: listB )
        {
            ret = mat.invert() * ret;
            node_map[id].push_back(key); // update node cache reference
        }
        // update node cache reference
        int32_t dst_id = G->get_node(dest).value().id();
        node_map[dst_id].push_back(key);
        int32_t orig_id = G->get_node(orig).value().id();
        node_map[orig_id].push_back(key);
        // update cache
        cache[key] = ret;
    }
	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRANSFORM
////////////////////////////////////////////////////////////////////////////////////////
std::optional<QVec> InnerAPI::transform(const std::string &destId, const QVec &origVec, const std::string &origId)
{
    assert(origVec.size() == 3);
    auto tm = get_transformation_matrix(destId, origId);
    if(tm.has_value())
        return (tm.value() * origVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
    else
        return {};
}

std::optional<QVec> InnerAPI::transform( const std::string &destId, const std::string &origId)
{
    return transform(destId, QVec::vec3(0.f, 0.f, 0.f), origId);
}

std::optional<QVec> InnerAPI::transform_axis(const std::string &destId, const QVec &origVec, const std::string & origId)
{
    assert(origVec.size() == 6);
    auto tm = get_transformation_matrix(destId, origId);
    if(tm.has_value())
    {
        const QMat M = tm.value();
        const QVec a = (M * origVec.subVector(0,2).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
        const Rot3D R(origVec(3), origVec(4), origVec(5));

        const QVec b = (M.getSubmatrix(0,2,0,2)*R).extractAnglesR_min();
        QVec ret(6);
        ret(0) = a(0);
        ret(1) = a(1);
        ret(2) = a(2);
        ret(3) = b(0);
        ret(4) = b(1);
        ret(5) = b(2);
        return ret;
    }
    else
        return {};
}

std::optional<QVec> InnerAPI::transform_axis(const std::string &destId, const std::string & origId)
{
    return transform_axis(destId, QVec::vec6(0.f, 0.f, 0.f, 0.f, 0.f, 0.f), origId);
}

////////////////////////////////////////////////////////////////////////////////////////
////// SLOTS
////// used to remove cached transforms when node/edge changes
////////////////////////////////////////////////////////////////////////////////////////
void InnerAPI::add_or_assign_edge_slot(const uint64_t from, const uint64_t to, const std::string& edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerAPI::del_node_slot(const uint64_t id)
{
    remove_cache_entry(id);
}
void InnerAPI::del_edge_slot(const uint64_t from, const uint64_t to, const std::string &edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerAPI::remove_cache_entry(const uint64_t id)
{
    node_reference::iterator it = node_map.find(id);
    if(it != node_map.end())
    {
        for(key_transform key: it->second)
        {
            cache.erase(key);
        }
    }
    node_map.erase(id);
}
