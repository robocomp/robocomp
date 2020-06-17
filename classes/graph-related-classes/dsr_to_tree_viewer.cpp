#include "CRDT_graphviewer.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QTreeWidget>
#include "CRDT_graphnode.h"
#include "CRDT_graphedge.h"
#include "dsr_to_tree_viewer.h"

using namespace DSR ;

DSRtoTreeViewer::DSRtoTreeViewer(std::shared_ptr<CRDT::CRDTGraph> G_, QTreeWidget *parent) :  QTreeWidget(parent)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
    G = G_;
    this->setMinimumSize(400, 400);
    //this->setFrameShape(NoFrame);
	//this->adjustSize();
 	setMouseTracking(true);

    createGraph();

    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, this, &DSRtoTreeViewer::add_or_assign_node_SLOT);
	//connect(G.get(), &CRDT::CRDTGraph::update_edge_signal, this, &DSRtoGraphViewer::addEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoGraphViewer::delEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoGraphViewer::delNodeSLOT);
}

void DSRtoTreeViewer::createGraph()
{
	qDebug() << __FUNCTION__ << "Reading graph in Graph Viewer";
    try
    {
        auto map = G->getCopy();
		for(const auto &[k, node] : map)
		       add_or_assign_node_SLOT(k,  node.type());
		for(auto node : map) // Aworset
           	for(const auto &[k, edges] : node.second.fano())
			   add_or_assign_edge_SLOT(edges.from(), edges.to(), edges.type());
    }
	catch(const std::exception &e) { std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;}
}

//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////
void DSRtoTreeViewer::add_or_assign_node_SLOT(int id, const std::string &type)
{	

}

void DSRtoTreeViewer::add_or_assign_edge_SLOT(std::int32_t from, std::int32_t to, const std::string &edge_tag)
{
	
}

void DSRtoTreeViewer::del_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string &edge_tag)
{
    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;
	
}

void DSRtoTreeViewer::del_node_SLOT(int id)
{
    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;
   
}
//////////////////////////////////////////////////////////////////////////////////////
///// EVENTS
//////////////////////////////////////////////////////////////////////////////////////

void DSRtoTreeViewer::wheelEvent(QWheelEvent* event)
{
    
}

void DSRtoTreeViewer::resizeEvent(QResizeEvent* event)
{
    this->resize(event->size());
}