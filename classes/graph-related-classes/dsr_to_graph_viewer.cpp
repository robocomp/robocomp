#include "CRDT_graphviewer.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QTableWidget>
#include "CRDT_graphnode.h"
#include "CRDT_graphedge.h"
#include "dsr_to_graph_viewer.h"

using namespace DSR ;

DSRtoGraphViewer::DSRtoGraphViewer(std::shared_ptr<CRDT::CRDTGraph> G_, QWidget *parent) :  QGraphicsView(parent)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
    G = G_;
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	scene.setSceneRect(-200, -200, 400, 400);
	this->setScene(&scene);
    this->setCacheMode(QGraphicsView::CacheBackground);
	this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	this->setRenderHint(QPainter::Antialiasing);
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->setMinimumSize(400, 400);
	this->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	this->adjustSize();
 	setMouseTracking(true);
    this->viewport()->setMouseTracking(true);

    createGraph();

    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, this, &DSRtoGraphViewer::add_or_assign_node_SLOT);
	//connect(G.get(), &CRDT::CRDTGraph::update_edge_signal, this, &DSRtoGraphViewer::addEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoGraphViewer::delEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoGraphViewer::delNodeSLOT);
}

void DSRtoGraphViewer::createGraph()
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
void DSRtoGraphViewer::add_or_assign_node_SLOT(int id, const std::string &type)
{	
	//qDebug() << __FUNCTION__ << "node id " << id<<", type "<<QString::fromUtf8(type.c_str());
	GraphNode *gnode;														// CAMBIAR a sharer_ptr

    auto name_op = G->get_name_from_id(id);
    auto name = name_op.value_or("No_name");
	std::optional<Node> n = G->get_node(id);
    if (n.has_value()) {
        if (gmap.count(id) == 0)    // if node does not exist, create it
        {
            gnode = new GraphNode(std::shared_ptr<DSRtoGraphViewer>(this));  
            gnode->id_in_graph = id;
            gnode->setType(type);
			gnode->setTag(n.value().name() + " [" + std::to_string(n.value().id()) + "]");
            scene.addItem(gnode);
            gmap.insert(std::pair(id, gnode));
            // //left table filling only if it is new
            // tableWidgetNodes->setColumnCount(1);
            // tableWidgetNodes->setHorizontalHeaderLabels(QStringList{"type"});
            // tableWidgetNodes->verticalHeader()->setVisible(false);
            // tableWidgetNodes->setShowGrid(false);
            // nodes_types_list << QString::fromStdString(type);
            // nodes_types_list.removeDuplicates();
            // int i = 0;
            // tableWidgetNodes->clearContents();
            // tableWidgetNodes->setRowCount(nodes_types_list.size());
            // for (auto &s : nodes_types_list) 
			// {
            //     tableWidgetNodes->setItem(i, 0, new QTableWidgetItem(s));
            //     tableWidgetNodes->item(i, 0)->setIcon(QPixmap::fromImage(QImage("../../graph-related-classes/greenBall.png")));
            //     i++;
            // }
            // tableWidgetNodes->horizontalHeader()->setStretchLastSection(true);
            // tableWidgetNodes->resizeRowsToContents();
            // tableWidgetNodes->resizeColumnsToContents();
            // tableWidgetNodes->show();

            // connect QTableWidget itemClicked to hide/show nodes of selected type and nodes fanning into it
            // disconnect(tableWidgetNodes, &QTableWidget::itemClicked, nullptr, nullptr);
            // connect(tableWidgetNodes, &QTableWidget::itemClicked, this, [this](const auto &item) 
			// {
            //     static bool visible = true;
            //     std::cout << __FILE__ << " " << __FUNCTION__ << "hide or show all nodes of type " << item->text().toStdString() << std::endl;
            //     for (auto &[k, v] : gmap)
            //         if (item->text().toStdString() == v->getType()) {
            //             v->setVisible(!v->isVisible());
            //             for (const auto &gedge: gmap.at(k)->edgeList)
            //                 gedge->setVisible(!gedge->isVisible());
            //         }
            //     visible = !visible;
            //     if (visible)
            //         tableWidgetNodes->item(item->row(), 0)->setIcon(
            //                 QPixmap::fromImage(QImage("../../graph-related-classes/greenBall.png")));
            //     else
            //         tableWidgetNodes->item(item->row(), 0)->setIcon(
            //                 QPixmap::fromImage(QImage("../../graph-related-classes/redBall.png")));
            // }, Qt::UniqueConnection);
        
			std::string color = "coral";
			if(type == "world") color = "SeaGreen";
			else if(type == "transform") color = "SteelBlue";
			else if(type == "plane") color = "Khaki";
			else if(type == "differentialrobot") color = "GoldenRod";
			else if(type == "laser") color = "GreenYellow";
			else if(type == "mesh") color = "LightBlue";
			else if(type == "imu") color = "LightSalmon";
			gnode->setColor(color);
        } else
            gnode = gmap.at(id);

        float posx = 10;
        float posy = 10;
        try 
		{
            posx = G->get_attrib_by_name<float>(n.value(), "pos_x").value_or(10);
            posy = G->get_attrib_by_name<float>(n.value(), "pos_y").value_or(10);
        }
        catch (const std::exception &e) {
            auto rd = QVec::uniformVector(2, -200, 200);
            posx = rd.x();
            posy = rd.y();
        }
        if (posx != gnode->x() or posy != gnode->y())
            gnode->setPos(posx, posy);

        emit G->update_attrs_signal(id, n.value().attrs());
    }
}

void DSRtoGraphViewer::add_or_assign_edge_SLOT(std::int32_t from, std::int32_t to, const std::string &edge_tag)
{
	try 
    {
 		//qDebug() << __FUNCTION__ << "edge id " << QString::fromStdString(edge_tag) << from << to;
		std::tuple<std::int32_t, std::int32_t, std::string> key = std::make_tuple(from, to, edge_tag);

		if(gmap_edges.count(key) == 0) 
		{ 		
			//check if edge already exists
			auto node_origen = gmap.at(from);
			auto node_dest = gmap.at(to);
			auto item = new GraphEdge(node_origen, node_dest, edge_tag.c_str());
			scene.addItem(item);
			gmap_edges.insert(std::make_pair(key, item));
			// //side table filling
			// tableWidgetEdges->setColumnCount(1);
			// tableWidgetEdges->setHorizontalHeaderLabels(QStringList{"label"});
			// tableWidgetNodes->verticalHeader()->setVisible(false);
			// tableWidgetNodes->setShowGrid(false);
			// edges_types_list << QString::fromStdString(edge_tag);
			// edges_types_list.removeDuplicates();
			// int i = 0;
			// tableWidgetEdges->clearContents();
			// tableWidgetEdges->setRowCount(edges_types_list.size());
			// for (auto &s : edges_types_list) {
			// 	tableWidgetEdges->setItem(i, 0, new QTableWidgetItem(s));
			// 	tableWidgetEdges->item(i, 0)->setIcon(
			// 			QPixmap::fromImage(QImage("../../graph-related-classes/greenBall.png")));
			// 	i++;
			// }
			// tableWidgetEdges->horizontalHeader()->setStretchLastSection(true);
			// tableWidgetEdges->resizeRowsToContents();
			// tableWidgetEdges->resizeColumnsToContents();
			// tableWidgetEdges->show();
		}
	}
	catch(const std::exception &e) {
		std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<<" "<<e.what()<< std::endl;}
}

void DSRtoGraphViewer::del_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string &edge_tag)
{
    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;
	try {
		std::tuple<std::int32_t, std::int32_t, std::string> key = std::make_tuple(from, to, edge_tag);
		while (gmap_edges.count(key) > 0) {
            scene.removeItem(gmap_edges.at(key));
		    gmap_edges.erase(key);
		}
	} catch(const std::exception &e) { std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<< std::endl;}

}

void DSRtoGraphViewer::del_node_SLOT(int id)
{
    std::cout<<__FUNCTION__<<":"<<__LINE__<< std::endl;
    try {
        while (gmap.count(id) > 0) {
            scene.removeItem(gmap.at(id));
            gmap.erase(id);
        }
    } catch(const std::exception &e) { std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<< std::endl;}

}


//  void GraphViewer::NodeAttrsChangedSLOT(const std::int32_t &id, const DSR::Attribs &attribs)
//  {
// 	try 
// 	{
// 		std::cout << __FUNCTION__ << id << std::endl;
// 		float posx = std::get<float>(attribs.at("pos_x"));
// 		float posy = std::get<float>(attribs.at("pos_y"));
// 		auto &gnode = gmap.at(id);
// 		if(posx != gnode->x() or posy != gnode->y())
// 			gnode->setPos(posx, posy);
// 	}
// 	catch(const std::exception &e){ std::cout << "Exception: " << e.what() << " pos_x and pos_y attribs not found in node "  << id << std::endl;};
//  }



//////////////////////////////////////////////////////////////////////////////////////
///// EVENTS
//////////////////////////////////////////////////////////////////////////////////////

void DSRtoGraphViewer::wheelEvent(QWheelEvent* event)
{
    const QGraphicsView::ViewportAnchor anchor = transformationAnchor();
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	int angle = event->angleDelta().y();
	qreal factor;
	if (angle > 0) 
	{
		factor = 1.1;
		QRectF r = scene.sceneRect();
		scene.setSceneRect(r);
	}
	else
	{
		factor = 0.9;
		QRectF r = scene.sceneRect();
		scene.setSceneRect(r);
	}
	this->scale(factor, factor);
	this->setTransformationAnchor(anchor);
}

void DSRtoGraphViewer::resizeEvent(QResizeEvent *e)
{  
//	qDebug() << "resize_graph_view" << x() << y()<<e->size(); 
	this->resize(e->size());
} 