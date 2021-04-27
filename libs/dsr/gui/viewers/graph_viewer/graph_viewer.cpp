#include <dsr/gui/dsr_gui.h>
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QTableWidget>
#include <QApplication>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <dsr/gui/viewers/graph_viewer/graph_edge.h>
#include <dsr/gui/viewers/graph_viewer/graph_viewer.h>

using namespace DSR ;

GraphViewer::GraphViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent) :  AbstractGraphicViewer(parent)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::uint32_t>("std::uint32_t");
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    qRegisterMetaType<uint64_t>("uint64_t");
    qRegisterMetaType<std::string>("std::string");
    G = std::move(G_);
	own = std::shared_ptr<GraphViewer>(this);

    contextMenu = new QMenu(this);
    showMenu = contextMenu->addMenu(tr("&Show:"));

    createGraph();

	this->scene.setSceneRect(scene.itemsBoundingRect());

	this->fitInView(scene.itemsBoundingRect(), Qt::KeepAspectRatio );

	central_point = new QGraphicsEllipseItem(0,0,0,0);
	scene.addItem(central_point);

	connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &GraphViewer::add_or_assign_node_SLOT);
	connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &GraphViewer::add_or_assign_edge_SLOT);
	connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &GraphViewer::del_edge_SLOT);
	connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &GraphViewer::del_node_SLOT);
}


GraphViewer::~GraphViewer()
{
    qDebug() << __FUNCTION__ << "Destroy";
    qDebug()  << "GraphViewer: " << G.use_count();
    G.reset();
    gmap.clear();
	gmap_edges.clear();
    type_id_map.clear();
	QList<QGraphicsItem*> allGraphicsItems = scene.items();
	for(int i = 0; i < allGraphicsItems.size(); i++)
	{
		QGraphicsItem *graphicItem = allGraphicsItems[i];
		if(graphicItem->scene() == &scene)
			scene.removeItem(graphicItem);

	}
	scene.clear();
}

void GraphViewer::createGraph()
{
	gmap.clear();
	gmap_edges.clear();
    type_id_map.clear();
	this->scene.clear();
	qDebug() << __FUNCTION__ << "Reading graph in Graph Viewer";
    try
    {
        std::set<std::string> type_list;
        auto map = G->getCopy();
		for(const auto &[k, node] : map) {
            add_or_assign_node_SLOT(k, node.type());
            //context menu
            if (type_list.find(node.type()) == type_list.end()) {
                QAction *action = new QAction(QString::fromStdString(node.type()));
                action->setCheckable(true);
                action->setChecked(true);
                showMenu->addAction(action);
                std::string type = node.type();
                connect(action, &QAction::toggled, this, [this, type](bool visible){
                    std::cout<<"hide/show";
                    for(auto id : type_id_map[type])
                        hide_show_node_SLOT(id, visible);
                });
                type_list.insert(node.type());
                type_id_map.insert(std::pair<std::string, std::set<std::uint64_t>>(node.type(), {node.id()}));
            } else
                type_id_map[node.type()].insert(node.id());
        }
		for(auto node : map) // Aworset
           	for(const auto &[k, edges] : node.second.fano())
			   add_or_assign_edge_SLOT(edges.from(), edges.to(), edges.type());
    }
	catch(const std::exception &e) { std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;}
}


///////////////////////////////////////

void GraphViewer::toggle_animation(bool animate)
{
	qDebug() << "timerId " << timerId ;
	if(animate)
	{
		if(timerId == 0)
	   		timerId = startTimer(1000 / 25);
	}
	else
	{
		killTimer(timerId);
		timerId = 0;
	}
}

void GraphViewer::timerEvent(QTimerEvent *event)
{
	// Q_UNUSED(event)

	for( auto &[_,node] : gmap)
	{
		node->calculateForces();
	}
	bool itemsMoved = false;

	for( auto &[_,node] : gmap)
	{
		itemsMoved = node->advancePosition() or itemsMoved;
	}
	if (!itemsMoved)
	{
		killTimer(timerId);
		timerId = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////
void GraphViewer::add_or_assign_node_SLOT(uint64_t id, const std::string &type)
{
	//qDebug() << __FUNCTION__ << "node id " << id<<", type "<<QString::fromUtf8(type.c_str());
															// CAMBIAR a sharer_ptr
    GraphNode *gnode;
    auto name_op = G->get_name_from_id(id);
    auto name = name_op.value_or("No_name");
	std::optional<Node> n = G->get_node(id);
    if (n.has_value()) {
        if (gmap.count(id) == 0)    // if node does not exist, create it
        {
            qDebug()<<__FUNCTION__<<"##### New node";
            gnode = this->new_visual_node(id, type, name, false);
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
            //     tableWidgetNodes->item(i, 0)->setIcon(QPixmap::fromImage(QImage("../../dsr/greenBall.png")));
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
            //     qDebug() << __FILE__ << " " << __FUNCTION__ << "hide or show all nodes of type " << item->text().toStdString() ;
            //     for (auto &[k, v] : gmap)
            //         if (item->text().toStdString() == v->getType()) {
            //             v->setVisible(!v->isVisible());
            //             for (const auto &gedge: gmap.at(k)->edgeList)
            //                 gedge->setVisible(!gedge->isVisible());
            //         }
            //     visible = !visible;
            //     if (visible)
            //         tableWidgetNodes->item(item->row(), 0)->setIcon(
            //                 QPixmap::fromImage(QImage("../../dsr/greenBall.png")));
            //     else
            //         tableWidgetNodes->item(item->row(), 0)->setIcon(
            //                 QPixmap::fromImage(QImage("../../dsr/redBall.png")));
            // }, Qt::UniqueConnection);
		
            //color is read from node attribute
            std::string color("coral");
            color = G->get_attrib_by_name<color_att>(n.value()).value_or(color);
/* 
			std::string color = "coral";
			if(type == "world") color = "SeaGreen";
			else if(type == "transform") color = "SteelBlue";
			else if(type == "plane") color = "Khaki";
			else if(type == "differentialrobot") color = "GoldenRod";
			else if(type == "laser") color = "GreenYellow";
			else if(type == "mesh") color = "LightBlue";
			else if(type == "imu") color = "LightSalmon";*/
			gnode->setColor(color);
        }
        else
		{
			qDebug()<<__FUNCTION__<<"##### Updated node";
            gnode = gmap.at(id);
		}
		gnode->change_detected();
        float posx = 10;
        float posy = 10;
        try
        {
            posx = G->get_attrib_by_name<pos_x_att>(n.value()).value_or(10);
            posy = G->get_attrib_by_name<pos_y_att>(n.value()).value_or(10);
        }
        catch (const std::exception &e) {
            auto rd = QVec::uniformVector(2, -200, 200);
            posx = rd.x();
            posy = rd.y();
        }
        // Avoid to move if it's in the same position or if the node is grabbed
        if ((posx != gnode->x() or posy != gnode->y()) and gnode != scene.mouseGrabberItem()) {
			qDebug()<<__FUNCTION__<<"##### posx "<<posx<<" != gnode->x() "<<gnode->x()<<" or posy "<<posy<<" != gnode->y() "<<gnode->y();
			gnode->setPos(posx, posy);
		}

        emit G->update_attrs_signal(id, n.value().attrs());
    }
}

GraphNode* GraphViewer::new_visual_node(uint64_t id, const std::string &type, const std::string &name, bool debug)
{
    GraphNode *gnode = new GraphNode(own);
    gnode->id_in_graph = id;
    gnode->setType(type);
    std::string tag = name;
    tag += debug?" [" + std::to_string(id) + "]":"";
    gnode->setTag(tag);
    scene.addItem(gnode);
    return gnode;
}

void GraphViewer::add_or_assign_edge_SLOT(std::uint64_t from, std::uint64_t to, const std::string &edge_tag)
{
	try
    {
 		//qDebug() << __FUNCTION__ << "edge id " << QString::fromStdString(edge_tag) << from << to;
		std::tuple<std::uint64_t, std::uint64_t, std::string> key = std::make_tuple(from, to, edge_tag);

		if(gmap_edges.count(key) == 0) 
		{ 		

			auto item = this->new_visual_edge(from, to, edge_tag.c_str());
			gmap_edges.insert(std::make_pair(key, item));
		}
		gmap_edges[key]->change_detected();
	}
	catch(const std::exception &e) {
		std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<<" "<<e.what()<< std::endl;}

}

GraphEdge* GraphViewer::new_visual_edge(GraphNode *sourceNode, GraphNode *destNode, const QString &edge_name)
{
    auto gedge = new GraphEdge(sourceNode, destNode, edge_name);
    scene.addItem(gedge);
    return gedge;
}

GraphEdge* GraphViewer::new_visual_edge(std::uint64_t from, std::uint64_t to, const std::string &edge_tag)
{
    auto sourceNode = gmap.at(from);
    auto destNode = gmap.at(to);
    auto gedge = new GraphEdge(sourceNode, destNode, edge_tag.c_str());
    scene.addItem(gedge);
    return gedge;
}

void GraphViewer::del_edge_SLOT(const std::uint64_t from, const std::uint64_t to, const std::string &edge_tag)
{
    qDebug()<<__FUNCTION__<<":"<<__LINE__;
	try {
		std::tuple<std::uint64_t, std::uint64_t, std::string> key = std::make_tuple(from, to, edge_tag);
		while (gmap_edges.count(key) > 0) {
            GraphEdge *edge = gmap_edges.at(key);
            scene.removeItem(edge);
            if (gmap.find(from) != gmap.end())
                gmap.at(from)->deleteEdge(edge);
            if (gmap.find(to) != gmap.end())
                gmap.at(to)->deleteEdge(edge);
            gmap_edges.erase(key);
            delete edge;
		}
	} catch(const std::exception &e) { std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<< std::endl;}

}


void GraphViewer::del_node_SLOT(int id)
{
    qDebug()<<__FUNCTION__<<":"<<__LINE__;
    try {
        while (gmap.count(id) > 0) {
        	auto item = gmap.at(id);
            scene.removeItem(item);
            delete item;
            gmap.erase(id);

        }
    } catch(const std::exception &e) { std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<< std::endl;}

}

void GraphViewer::hide_show_node_SLOT(int id, bool visible)
{
	auto item = gmap[id];
	item->setVisible(visible);
	for (const auto &gedge: item->edgeList)
	{
		if((visible and gedge->destNode()->isVisible() and gedge->sourceNode()->isVisible()) or !visible)
		{
			gedge->setVisible(visible);
		}
	}

}

void GraphViewer::mousePressEvent(QMouseEvent *event)
{
	auto item = this->scene.itemAt(mapToScene(event->pos()), QTransform());
	if(item) {
		QGraphicsView::mousePressEvent(event);
	}
	else {
        if (event->button() == Qt::RightButton)
            showContextMenu(event);
		else
            AbstractGraphicViewer::mousePressEvent(event);
	}
}

void GraphViewer::reload(QWidget * widget)
{
	if(qobject_cast<GraphViewer*>(widget) != nullptr)
	{
		createGraph();
	}
}

void GraphViewer::showContextMenu(QMouseEvent *event)
{
    contextMenu->exec(event->globalPos());
}
