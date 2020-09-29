#include "dsr_inner_eigen_api.h"
#include "dsr_api.h"

using namespace DSR;


//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
InnerEigenAPI::InnerEigenAPI(DSR::DSRGraph *G_)
{
    G = G_;
    //update signals
    connect(G, &DSR::DSRGraph::update_node_signal, this, &InnerEigenAPI::add_or_assign_node_slot);
    connect(G, &DSR::DSRGraph::update_edge_signal, this, &InnerEigenAPI::add_or_assign_edge_slot);
    connect(G, &DSR::DSRGraph::del_edge_signal, this, &InnerEigenAPI::del_edge_slot);
    connect(G, &DSR::DSRGraph::del_node_signal, this, &InnerEigenAPI::del_node_slot);
}

/// Computation of resultant RTMat going from A to common ancestor and from common ancestor to B (inverted)
std::optional<InnerEigenAPI::Lists> InnerEigenAPI::setLists(const std::string &destId, const std::string &origId)
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
		auto edge_rt = G->get_edge_RT(p_node.value(), a.id()).value();
		auto rtmat = G->get_edge_RT_as_rtmat(edge_rt).value();
    	listA.emplace_back(std::make_tuple(p_node.value().id(), std::move(rtmat)));   // the downwards RT link from parent to a
        a = p_node.value();
	}
	while (G->get_node_level(b).value_or(-1) >= minLevel)
	{
        //qDebug() << "listaB" << b.id() << G->get_node_level(b).value() << G->get_parent_id(b).value();
		auto p_node = G->get_parent_node(b);
		if(not p_node.has_value())
			break;
		auto edge_rt = G->get_edge_RT(p_node.value(), b.id()).value();
		auto rtmat = G->get_edge_RT_as_rtmat(edge_rt).value();
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
	  		listA.push_back(std::make_tuple(p.value().id(), G->get_edge_RT_as_rtmat(G->get_edge_RT(p.value(), a.id()).value()).value()));
	  		listB.push_front(std::make_tuple(q.value().id(), G->get_edge_RT_as_rtmat(G->get_edge_RT(q.value(), b.id()).value()).value()));
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

std::optional<Mat::RTMat> InnerEigenAPI::getTransformationMatrixS(const std::string &dest, const std::string &orig)
{
	Mat::RTMat ret(Mat::RTMat::Identity());
    key_transform key = std::make_tuple(dest, orig);
    if( transform_cache::iterator it = cache.find(key) ; it != cache.end())
        ret = it->second;
    else
	{
        auto lists = setLists(dest, orig);
        if(!lists.has_value())
            return {};
        auto &[listA, listB] = lists.value();

        for(auto &[id, mat]: listA )
        {
            ret = mat * ret;
            node_map[id].push_back(key); // update node cache reference
        }
        for(auto &[id, mat]: listB )
        {
            ret = mat.inverse() * ret;
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

std::optional<Mat::RTMat> InnerEigenAPI::getTransformationMatrix(const QString &dest, const QString &orig)
{
	return getTransformationMatrixS(dest.toStdString(), orig.toStdString());
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRANSFORM
////////////////////////////////////////////////////////////////////////////////////////
std::optional<Eigen::VectorXd> InnerEigenAPI::transformS(const std::string &destId, const Eigen::VectorXd &initVec, const std::string &origId)
{

    //std::cout <<__FUNCTION__ << " " << initVec << std::endl;
	if (initVec.size()==3)
	{
		auto tm = getTransformationMatrixS(destId, origId);
        //std::cout << __FUNCTION__ << " " << tm.value().matrix().format(CleanFmt) << std::endl;
		if(tm.has_value())
        {
            //qInfo() << __FUNCTION__ << " " << tm.value().matrix().size() << initVec.homogeneous().size();
            return (tm.value() * initVec.homogeneous());
        }
		else
			return {};
	}
	else if (initVec.size()==6)
	{
		auto tm = getTransformationMatrixS(destId, origId);
		if(tm.has_value())
		{
			const Mat::RTMat rtmat = tm.value();
			const Eigen::Vector3d a = rtmat * (initVec.head(3).homogeneous());
			const Mat::Rot3D R(Eigen::AngleAxisd(initVec(3), Eigen::Vector3d::UnitX()) *
			                   Eigen::AngleAxisd(initVec(4), Eigen::Vector3d::UnitY()) *
			                   Eigen::AngleAxisd(initVec(5), Eigen::Vector3d::UnitZ()));
	         const Eigen::Vector3d b = ( rtmat.rotation() * R).eulerAngles(0,1,2);
            Mat::Vector6d ret;
			ret << a(0),  a(1), a(2), b(0), b(1), b(2);
			return ret;
		}
		else
		return {};
	}
	else
		return {};
}

 std::optional<Eigen::VectorXd> InnerEigenAPI::transformS( const std::string &destId, const std::string &origId)
 {
	return transformS(destId, Eigen::Vector3d(0.,0.,0.), origId);
 }

 std::optional<Eigen::VectorXd> InnerEigenAPI::transform(const QString & destId, const Eigen::VectorXd &origVec, const QString & origId)
 {
	return transformS(destId.toStdString(), origVec, origId.toStdString());
 }

 std::optional<Eigen::VectorXd> InnerEigenAPI::transform( const QString &destId, const QString & origId)
 {
 	return transformS(destId.toStdString(), Eigen::Vector3d(0.,0.,0.), origId.toStdString());
 }

std::optional<Eigen::VectorXd> InnerEigenAPI::transform6D( const QString &destId, const QString & origId)
 {
    Mat::Vector6d v;
	return transformS(destId.toStdString(), v.Zero(), origId.toStdString());
 }

std::optional<Eigen::VectorXd> InnerEigenAPI::transform6D( const QString &destId, const Mat::Vector6d &origVec, const QString & origId)
 {
	return transformS(destId.toStdString(), origVec, origId.toStdString());
 }

 std::optional<Eigen::VectorXd> InnerEigenAPI::transformS6D( const std::string &destId, const Mat::Vector6d &origVec, const std::string& origId)
 {
	Q_ASSERT(origVec.size() == 6);
	return transformS(destId, origVec, origId);
 }

 std::optional<Eigen::VectorXd> InnerEigenAPI::transformS6D( const std::string &destId, const std::string & origId)
 {
     Mat::Vector6d v;
     return transformS(destId, v.Zero(), origId);
 }

////////////////////////////////////////////////////////////////////////
/// SLOTS ==> used to remove cached transforms when node/edge changes
///////////////////////////////////////////////////////////////////////
void InnerEigenAPI::add_or_assign_node_slot(const std::int32_t id, const std::string &type)
{

}
void InnerEigenAPI::add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerEigenAPI::del_node_slot(const std::int32_t id)
{
    remove_cache_entry(id);
}
void InnerEigenAPI::del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerEigenAPI::remove_cache_entry(const std::int32_t id)
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