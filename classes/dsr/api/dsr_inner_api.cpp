
#include "dsr_inner_api.h"
#include "dsr_api.h"

using namespace DSR;

//InnerAPI::InnerAPI(std::shared_ptr<DSR::DSRGraph> _G)
InnerAPI::InnerAPI(DSR::DSRGraph *G_)
{
    G = G_;
}

/// Computation of resultant RTMat going from A to common ancestor and from common ancestor to B (inverted)
std::optional<InnerAPI::Lists> InnerAPI::setLists(const std::string &destId, const std::string &origId)
{
    std::list<RMat::RTMat> listA, listB;

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
		auto rtmat = G->get_edge_RT_as_RTMat(edge_rt).value();
		listA.emplace_back(std::move(rtmat));   // the downwards RT link from parent to a
        a = p_node.value();
	}
	while (G->get_node_level(b).value_or(-1) >= minLevel)
	{
//qDebug() << "listaB" << b.id() << G->get_node_level(b).value() << G->get_parent_id(b).value();
		auto p_node = G->get_parent_node(b);
		if(not p_node.has_value())
			break;
		auto edge_rt = G->get_edge_RT(p_node.value(), b.id()).value();
		auto rtmat = G->get_edge_RT_as_RTMat(edge_rt).value();
        listB.emplace_front(std::move(rtmat));
		b = p_node.value();
	}
	while (a.id() != b.id())
	{
		auto p = G->get_node(G->get_parent_id(a).value_or(-1));
		auto q = G->get_node(G->get_parent_id(b).value_or(-1));
		if(p.has_value() and q.has_value())
		{
//qDebug() << "listas A&B" << p.value().id() << q.value().id();
	  		listA.push_back(G->get_edge_RT_as_RTMat(G->get_edge_RT(p.value(), a.id()).value()).value());
	  		listB.push_front(G->get_edge_RT_as_RTMat(G->get_edge_RT(p.value(), b.id()).value()).value());
			a = p.value();
			b = q.value();
		}
		else
			return {};
	}
    return std::make_tuple(listA, listB);
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRNASFORMATION MATRIX
////////////////////////////////////////////////////////////////////////////////////////

std::optional<RTMat> InnerAPI::getTransformationMatrixS(const std::string &dest, const std::string &orig)
{
	RTMat ret;
	// if (localHashTr.contains(QPair<QString, QString>(to, from)))
	// {
	// 	ret = localHashTr[QPair<QString, QString>(to, from)];
	// }
	// else
	// {

    auto lists = setLists(dest, orig);
	if(!lists.has_value())
		return {};
	auto &[listA, listB] = lists.value();

    for(auto &a: listA )
    {
	    ret = a*ret;
        //ret.print("ListA");
    }
    for(auto &b: listB )
    {
        ret = b.invert() * ret;
        //ret.print("ListB");
    }
    //	localHashTr[QPair<QString, QString>(to, from)] = ret;
    //}
	return ret;
}

std::optional<RTMat> InnerAPI::getTransformationMatrix(const QString &dest, const QString &orig)
{
	return getTransformationMatrixS(dest.toStdString(), orig.toStdString());
}

////////////////////////////////////////////////////////////////////////////////////////
////// TRNASFORM
////////////////////////////////////////////////////////////////////////////////////////
std::optional<QVec> InnerAPI::transformS(const std::string &destId, const QVec &initVec, const std::string &origId)
{
	if (initVec.size()==3)
	{
		auto tm = getTransformationMatrixS(destId, origId);
		if(tm.has_value())
			return (tm.value() * initVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
		else
			return {};
	}
	else if (initVec.size()==6)
	{
		auto tm = getTransformationMatrixS(destId, origId);
		if(tm.has_value())
		{
			const QMat M = tm.value();
			const QVec a = (M * initVec.subVector(0,2).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			const Rot3D R(initVec(3), initVec(4), initVec(5));

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
	else
		return {};
}

 std::optional<QVec> InnerAPI::transformS( const std::string &destId, const std::string &origId)
 {
	return transformS(destId, QVec::vec3(0.,0.,0.), origId);
 }

 std::optional<QVec> InnerAPI::transform(const QString & destId, const QVec &origVec, const QString & origId)
 {
	return transformS(destId.toStdString(), origVec, origId.toStdString());
 }

 std::optional<QVec> InnerAPI::transform( const QString &destId, const QString & origId)
 {
 	return transformS(destId.toStdString(), QVec::vec3(0.,0.,0.), origId.toStdString());
 }

std::optional<QVec> InnerAPI::transform6D( const QString &destId, const QString & origId)
 {
	return transformS(destId.toStdString(), QVec::vec6(0,0,0,0,0,0), origId.toStdString());
 }

std::optional<QVec> InnerAPI::transform6D( const QString &destId, const QVec &origVec, const QString & origId)
 {
	Q_ASSERT(origVec.size() == 6);
	return transformS(destId.toStdString(), origVec, origId.toStdString());
 }

 std::optional<QVec> InnerAPI::transformS6D( const std::string &destId, const QVec &origVec, const std::string& origId)
 {
	Q_ASSERT(origVec.size() == 6);
	return transformS(destId, origVec, origId);
 }

 std::optional<QVec> InnerAPI::transformS6D( const std::string &destId, const std::string & origId)
 {
	return transformS(destId, QVec::vec6(0,0,0,0,0,0), origId);
 }