#include <dsr/api/dsr_inner_eigen_api.h>
#include <dsr/api/dsr_api.h>

using namespace DSR;

//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
InnerEigenAPI::InnerEigenAPI(DSR::DSRGraph *G_)
{
    G = G_;
    rt = G->get_rt_api();
    //update signals
    connect(G, &DSR::DSRGraph::update_edge_signal, this, &InnerEigenAPI::add_or_assign_edge_slot);
    connect(G, &DSR::DSRGraph::del_edge_signal, this, &InnerEigenAPI::del_edge_slot);
    connect(G, &DSR::DSRGraph::del_node_signal, this, &InnerEigenAPI::del_node_slot);
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRANSFORMATION MATRIX
////////////////////////////////////////////////////////////////////////////////////////

std::optional<Mat::RTMat> InnerEigenAPI::get_transformation_matrix(const std::string &dest, const std::string &orig, std::uint64_t timestamp)
{
   KeyTransform key = std::make_tuple(dest, orig);
    if( auto it = cache.find(key) ; it != cache.end())
        return it->second;
    else
    {
        Mat::RTMat atotal(Mat::RTMat::Identity());
        Mat::RTMat btotal(Mat::RTMat::Identity());

        auto an = G->get_node(orig);
        auto bn = G->get_node(dest);
        if ( not an.has_value() or  not bn.has_value())
        {
            qWarning() << __FUNCTION__ << ":"<<__LINE__<< " origen or dest nodes do not exist: " << QString::fromStdString(orig) << QString::fromStdString(dest);
            return {};
        }
        auto a = an.value(); auto b = bn.value();
        int minLevel = std::min(G->get_node_level(a).value_or(-1), G->get_node_level(b).value_or(-1));
        if (minLevel == -1)
        {
            qWarning() << __FUNCTION__ << ":"<<__LINE__ << " Incorrect level in one origin or dest nodes: " << QString::fromStdString(orig) << QString::fromStdString(dest) << ". The nodes may not have RT edges";
            return {};
        }
        while (G->get_node_level(a).value_or(-1) >= minLevel)
        {
            //qDebug() << "listaA" << a.id() << G->get_node_level(a).value() << G->get_parent_id(a).value();
            auto p_node = G->get_parent_node(a);
            if( not p_node.has_value())
                break;
            auto edge_rt = rt->get_edge_RT(p_node.value(), a.id());
            if (not edge_rt.has_value())
            {
                qWarning() << __FUNCTION__ << ":"<<__LINE__<< " Cannot find RT edge between Parent (" << QString::fromStdString(p_node->name()) << ", " << p_node->id() <<") and son (" << QString::fromStdString(a.name()) << ", " << a.id() <<") nodes going from: " << QString::fromStdString(orig) << " to: " << QString::fromStdString(dest);
                return {};
            }
            if( auto rtmat = rt->get_edge_RT_as_rtmat(edge_rt.value(), timestamp); rtmat.has_value())
            {
                atotal = rtmat.value() * atotal;
                node_map[p_node.value().id()].push_back(key); // update node cache reference
                a = p_node.value();
            }
            else return {};
        }
        while (G->get_node_level(b).value_or(-1) >= minLevel)
        {
            //qDebug() << "listaB" << b.id() << G->get_node_level(b).value() << G->get_parent_id(b).value();
            auto p_node = G->get_parent_node(b);
            if(not p_node.has_value())
                break;
            auto edge_rt = rt->get_edge_RT(p_node.value(), b.id());
            if (not edge_rt.has_value())
            {
                qWarning() << __FUNCTION__ << ":"<<__LINE__ << " Cannot find RT edge between Parent (" << QString::fromStdString(p_node->name()) << ", " << p_node->id() <<") and son (" << QString::fromStdString(a.name()) << ", " << a.id() <<") nodes going from: " << QString::fromStdString(orig) << " to: " << QString::fromStdString(dest);
                return {};
            }
            if( auto rtmat = rt->get_edge_RT_as_rtmat(edge_rt.value(), timestamp); rtmat.has_value())
            {
                btotal = rtmat.value() * btotal;
                node_map[p_node.value().id()].push_back(key); // update node cache reference
                b = p_node.value();
            }
            else
                return {};
        }
        // from min_level up tp the common ancestor
        while (a.id() != b.id())
        {
            auto p_node = G->get_node(G->get_parent_id(a).value_or(-1));
            auto q_node = G->get_node(G->get_parent_id(b).value_or(-1));
            if(p_node.has_value() and q_node.has_value())
            {
                //qDebug() << "listas A&B" << p_node.value().id() << q_node.value().id();
                auto a_edge_rt = rt->get_edge_RT(p_node.value(), a.id());
                if (not a_edge_rt.has_value())
                {
                    qWarning() << __FUNCTION__ << ":"<<__LINE__ << " Cannot find RT edge between Parent (" << QString::fromStdString(p_node->name()) << ", " << p_node->id() <<") and son (" << QString::fromStdString(a.name()) << ", " << a.id() <<") nodes going from: " << QString::fromStdString(orig) << " to: " << QString::fromStdString(dest);
                    return {};
                }
                auto b_edge_rt = rt->get_edge_RT(q_node.value(), b.id());
                if (not b_edge_rt.has_value())
                {
                    qWarning() << __FUNCTION__ << ":"<<__LINE__ << " Cannot find RT edge between Parent (" << QString::fromStdString(p_node->name()) << ", " << p_node->id() <<") and son (" << QString::fromStdString(a.name()) << ", " << a.id() <<") nodes going from: " << QString::fromStdString(orig) << " to: " << QString::fromStdString(dest);
                    return {};
                }
                auto a_rtmat = rt->get_edge_RT_as_rtmat(a_edge_rt.value(), timestamp);
                auto b_rtmat = rt->get_edge_RT_as_rtmat(b_edge_rt.value(), timestamp);
                if(a_rtmat.has_value() and b_rtmat.has_value())
                {
                    atotal = a_rtmat.value() * atotal;
                    btotal = b_rtmat.value() * btotal;
                    node_map[p_node.value().id()].push_back(key); // update node cache reference
                    node_map[q_node.value().id()].push_back(key); // update node cache reference
                    a = p_node.value();
                    b = q_node.value();
                }
                else return {};
            }
            else
            {
                qWarning() << __FUNCTION__ << ":"<<__LINE__<< " non existing nodes while merging to common ancestor" << QString::fromStdString(a.name()) << QString::fromStdString(b.name());
                return {};
            }
        }
        // update node cache reference
        uint64_t dst_id = G->get_node(dest).value().id();
        node_map[dst_id].push_back(key);
        uint64_t orig_id = G->get_node(orig).value().id();
        node_map[orig_id].push_back(key);

        // update cache
        auto ret = btotal.inverse() * atotal;
        cache[key] = ret;
        return ret;
    }
}

std::optional<Mat::Rot3D> InnerEigenAPI::get_rotation_matrix(const std::string &dest, const std::string &orig, std::uint64_t timestamp)
{
    if( auto r = get_transformation_matrix(dest, orig, timestamp); r.has_value())
        return r.value().rotation();
    else
    {
        qWarning() << __FUNCTION__ << " Not able to compute transformation matrix between " << QString::fromStdString(orig) << " and " << QString::fromStdString(dest);
        return {};
    }
}

std::optional<Mat::Vector3d> InnerEigenAPI::get_translation_vector(const std::string &dest, const std::string &orig, std::uint64_t timestamp)
{
    if (auto r = get_transformation_matrix(dest, orig, timestamp); r.has_value())
        return r.value().translation();
    else
    {
        qWarning() << __FUNCTION__ << " Not able to compute transformation matrix between "
                   << QString::fromStdString(orig) << " and " << QString::fromStdString(dest);
        return {};
    }
}
std::optional<Mat::Vector3d> InnerEigenAPI::get_euler_xyz_angles(const std::string &dest, const std::string &orig, std::uint64_t timestamp)
{
    if( auto r = get_transformation_matrix(dest, orig, timestamp); r.has_value())
        return r.value().rotation().eulerAngles(0,1,2);  //X Y Z order
    else
    {
        qWarning() << __FUNCTION__ << " Not able to compute transformation matrix between " << QString::fromStdString(orig) << " and " << QString::fromStdString(dest);
        return {};
    }
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRANSFORM
////////////////////////////////////////////////////////////////////////////////////////
std::optional<Mat::Vector3d> InnerEigenAPI::transform(const std::string &dest, const Mat::Vector3d &vector, const std::string &orig, std::uint64_t timestamp)
{
    //std::cout <<__FUNCTION__ << " " << initVec << std::endl;
    auto tm = get_transformation_matrix(dest, orig, timestamp);
    //std::cout << __FUNCTION__ << " " << tm.value().matrix().format(CleanFmt) << std::endl;
    if(tm.has_value())
    {
        //qInfo() << __FUNCTION__ << " " << tm.value().matrix().size() << initVec.homogeneous().size();
        return (tm.value() * vector.homogeneous());
    }
    else
        return {};
}

std::optional<Mat::Vector3d> InnerEigenAPI::transform( const std::string &dest, const std::string & orig, std::uint64_t timestamp)
 {
 	return transform(dest, Mat::Vector3d(0.,0.,0.), orig, timestamp);
 }

std::optional<Mat::Vector6d> InnerEigenAPI::transform_axis(const std::string &dest, const Mat::Vector6d &vector, const std::string &orig, std::uint64_t timestamp)
{
    auto tm = get_transformation_matrix(dest, orig, timestamp);
    if(tm.has_value())
    {
        const Mat::RTMat rtmat = tm.value();
        const Mat::Vector3d a = rtmat * (vector.head(3).homogeneous());
        const Mat::Rot3D r_axis(Eigen::AngleAxisd(vector(3), Mat::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(vector(4), Mat::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(vector(5), Mat::Vector3d::UnitZ()));
        // multiply orig-dest rotation by rotation matrix of oriented point
        const Mat::Vector3d b = ( rtmat.rotation() * r_axis).eulerAngles(0,1,2);
        Mat::Vector6d ret;
        ret << a(0), a(1), a(2), b(0), b(1), b(2);
        return ret;
    }
    else
        return {};
 }

std::optional<Mat::Vector6d> InnerEigenAPI::transform_axis( const std::string &dest,  const std::string & orig, std::uint64_t timestamp)
{
    Mat::Vector6d v;
	return transform_axis(dest, Mat::Vector6d::Zero(), orig, timestamp);
}

////////////////////////////////////////////////////////////////////////
/// SLOTS ==> used to remove cached transforms when node/edge changes
///////////////////////////////////////////////////////////////////////
void InnerEigenAPI::add_or_assign_edge_slot(uint64_t from, uint64_t to, const std::string& edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerEigenAPI::del_node_slot(uint64_t id)
{
    remove_cache_entry(id);
}
void InnerEigenAPI::del_edge_slot(uint64_t from, uint64_t to, const std::string &edge_type)
{
    if(edge_type == "RT")
    {
        remove_cache_entry(from);
        remove_cache_entry(to);
    }
}
void InnerEigenAPI::remove_cache_entry(uint64_t id)
{
    auto it = node_map.find(id);
    if(it != node_map.end())
    {
        for(const KeyTransform& key: it->second)
        {
            cache.erase(key);
        }
    }
    node_map.erase(id);
}

/////////////////////

///// Computation of resultant RTMat going from A to common ancestor and from common ancestor to B (inverted)
//std::optional<InnerEigenAPI::Lists> InnerEigenAPI::setLists(const std::string &dest, const std::string &orig)
//{
//    std::list<NodeMatrix> listA, listB;
//    Mat::RTMat atotal(Mat::RTMat::Identity());
//    Mat::RTMat btotal(Mat::RTMat::Identity());
//
//    auto an = G->get_node(orig);
//    auto bn = G->get_node(dest);
//    if ( not an.has_value() or  not bn.has_value())
//        return {};
//    auto a = an.value(); auto b = bn.value();
//
//    int minLevel = std::min(G->get_node_level(a).value_or(-1), G->get_node_level(b).value_or(-1));
//    if (minLevel == -1)
//        return {};
//    while (G->get_node_level(a).value_or(-1) >= minLevel)
//    {
//        qDebug() << "listaA" << a.id() << G->get_node_level(a).value() << G->get_parent_id(a).value();
//        auto p_node = G->get_parent_node(a);
//        if( not p_node.has_value())
//            break;  // FIX THIS GIVING MORE INFO
//        auto edge_rt = G->get_edge_RT(p_node.value(), a.id()).value();
//        auto rtmat = G->get_edge_RT_as_rtmat(edge_rt).value();
//        atotal = rtmat * atotal;
//        listA.emplace_back(std::make_tuple(p_node.value().id(), std::move(rtmat)));   // the downwards RT link from parent to a
//        a = p_node.value();
//    }
//    while (G->get_node_level(b).value_or(-1) >= minLevel)
//    {
//        qDebug() << "listaB" << b.id() << G->get_node_level(b).value() << G->get_parent_id(b).value();
//        auto p_node = G->get_parent_node(b);
//        if(not p_node.has_value())
//            break;
//        auto edge_rt = G->get_edge_RT(p_node.value(), b.id()).value();
//        auto rtmat = G->get_edge_RT_as_rtmat(edge_rt).value();
//        btotal = rtmat.inverse() * btotal;
//        listB.emplace_front(std::make_tuple(p_node.value().id(), std::move(rtmat)));
//        b = p_node.value();
//    }
//    // from min_level up tp the common ancestor
//    while (a.id() != b.id())
//    {
//        auto p_node = G->get_node(G->get_parent_id(a).value_or(-1));
//        auto q_node = G->get_node(G->get_parent_id(b).value_or(-1));
//        if(p_node.has_value() and q_node.has_value())
//        {
//            qDebug() << "listas A&B" << p_node.value().id() << q_node.value().id();
//            auto a_edge_rt = G->get_edge_RT(p_node.value(), a.id()).value();
//            auto b_edge_rt = G->get_edge_RT(q_node.value(), b.id()).value();
//            auto a_rtmat = G->get_edge_RT_as_rtmat(a_edge_rt).value();
//            auto b_rtmat = G->get_edge_RT_as_rtmat(b_edge_rt).value();
//            atotal = a_rtmat * atotal;
//            btotal = b_rtmat.inverse() * btotal;
//            listA.emplace_back(std::make_tuple(p_node.value().id(), std::move(a_rtmat)));
//            listB.emplace_front(std::make_tuple(q_node.value().id(), std::move(b_rtmat)));
//            a = p_node.value();
//            b = q_node.value();
//        }
//        else
//            return {};
//    }
//    //	std::cout << "----QAUDUD--------" << std::endl;
//    //	std::cout << (atotal*btotal).matrix() << std::endl;
//    return std::make_tuple(listA, listB);
//}

//std::optional<Mat::RTMat> InnerEigenAPI::get_transformation_matrix(const std::string &dest, const std::string &orig)
//{
//	Mat::RTMat ret(Mat::RTMat::Identity());
//   KeyTransform key = std::make_tuple(dest, orig);
//    if( TransformCache::iterator it = cache.find(key) ; it != cache.end())
//        ret = it->second;
//    else
//	{
//        auto lists = setLists(dest, orig);
//        if(!lists.has_value())
//            return {};
//        auto &[listA, listB] = lists.value();
//
//        for(auto &[id, mat]: listA )
//        {
//            ret = mat * ret;
//            node_map[id].push_back(key); // update node cache reference
//        }
//        Mat::RTMat retB(Mat::RTMat::Identity());
//        for(auto &[id, mat]: listB )
//        {
//            //ret = mat.inverse() * ret;
//            retB = mat * retB;
//            node_map[id].push_back(key); // update node cache reference
//        }
//        ret = retB.inverse() * ret;
//        std::cout << "----ORIG--------" << std::endl;
//        std::cout << ret.matrix() << std::endl;
//        // update node cache reference
//        int32_t dst_id = G->get_node(dest).value().id();
//        node_map[dst_id].push_back(key);
//        int32_t orig_id = G->get_node(orig).value().id();
//        node_map[orig_id].push_back(key);
//
//        // update cache
//        cache[key] = ret;
//    }
//	return ret;
//}