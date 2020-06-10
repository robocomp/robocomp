#include "dsr_to_graphicscene_viewer.h"

using namespace DSR ;

DSRtoGraphicsceneViewer::DSRtoGraphicsceneViewer(std::shared_ptr<CRDT::CRDTGraph> G_, float scaleX, float scaleY, QGraphicsView *parent) : QGraphicsView(parent)
{
    G = G_;
    m_scaleX = scaleX;
    m_scaleY = scaleY;
    this->setMinimumSize(400,400);

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(-5000, -5000, 10000, 10000);
	this->scale(1, -1);
    this->setScene(&scene);
    this->setCacheMode(QGraphicsView::CacheBackground);
	this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	this->setRenderHint(QPainter::Antialiasing);
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	
 	setMouseTracking(true);
    this->viewport()->setMouseTracking(true);

    //center position
    scene.addRect(-100, -100, 200, 200, QPen(QColor("black")),QBrush(QColor("black")));
    //edge => x red, z (y) blue
    scene.addRect(-3000, -3000, 1000, 30, QPen(QColor("red")),QBrush(QColor("red")));
    scene.addRect(-3000, -3000, 30, 1000, QPen(QColor("blue")),QBrush(QColor("blue")));

    //initial graph
    createGraph();

    //update signals
    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_node_slot);
	connect(G.get(), &CRDT::CRDTGraph::update_edge_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_edge_slot);

	//connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoGraphicsceneViewer::delEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoGraphicsceneViewer::delNodeSLOT);

}

void DSRtoGraphicsceneViewer::createGraph()
{
    innermodel = G->get_inner_api();
    try
    {
        auto map = G->getCopy();
		for(const auto &[k, node] : map)
		    add_or_assign_node_slot(k,  node.type());
    }
	catch(const std::exception &e)
    {
        std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////

void DSRtoGraphicsceneViewer::add_or_assign_node_slot(const std::int32_t id, const std::string &type)
{
qDebug() << __FUNCTION__ ;
qDebug()<<"*************************";
    
    auto node = G->get_node(id);
std::cout << node.value().name() << " " << node.value().id() << std::endl;
    if(node.has_value())
    { 
        if( type == "plane" )
            add_or_assign_box(node.value());
        if( type == "mesh")
            add_or_assign_mesh(node.value());
    }
}
void DSRtoGraphicsceneViewer::add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type)
{
qDebug() << __FUNCTION__ ;
std::cout << "******UPDATE EDGE "<<from << " " << to << std::endl;
    std::string edge_key = std::to_string(from) + "_" + std::to_string(to);
    for (int node_id : edgeMap[edge_key])
    {
std::cout<<"update node " << node_id<<std::endl;
        update_scene_rect_pose(node_id);
    }
}

void DSRtoGraphicsceneViewer::add_or_assign_box(Node &node)
{
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    std::string filename = G->get_attrib_by_name<std::string>(node, "texture").value_or("");
    auto width = G->get_attrib_by_name<std::int32_t>(node, "width");
    auto height = G->get_attrib_by_name<std::int32_t>(node, "height");


    if(width.has_value()) std::cout << "width:" << width.value() << std::endl;    
    if(height.has_value()) std::cout << "height: " << height.value() << std::endl;


    //check if has required values
    if(width.has_value() and height.has_value())
    {
        add_or_assign_object(node.id(), width.value(), height.value(), node.name(), color, filename); 
    }
    else
    {
        std::cout<<"Error drawing "<< node << " width or height required attribs has no value"<<std::endl;
    }
}

void  DSRtoGraphicsceneViewer::add_or_assign_mesh(Node &node)
{   
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    std::string filename = G->get_attrib_by_name<std::string>(node, "path").value_or("");
    auto scalex = G->get_attrib_by_name<std::int32_t>(node, "scalex");
    auto scaley = G->get_attrib_by_name<std::int32_t>(node, "scaley");
    auto scalez = G->get_attrib_by_name<std::int32_t>(node, "scalez");

    if(scalex.has_value()) std::cout << scalex.value() << std::endl;
    if(scaley.has_value()) std::cout << scaley.value() << std::endl;
    if(scalez.has_value()) std::cout << scalez.value() << std::endl;

     //check if has required values
    if(scalex.has_value() and scaley.has_value() and scalez.has_value())
    {
        add_or_assign_object(node.id(), scalex.value(), scalez.value(), node.name(), color, filename);
    }
    else
    {
        std::cout<<"Error drawing "<< node << " scalex, scaley or scalez required attribs has no value"<<std::endl;
    }
}

void  DSRtoGraphicsceneViewer::add_or_assign_object(std::int32_t node_id, std::int32_t width, std::int32_t height, std::string node_name, std::string color, std::string filename)
{
    // get transfrom to world => to get correct position
    std::optional<QVec> pose = innermodel->transformS6D("world", node_name);
    if (pose.has_value())
    {
pose.value().print(QString::fromStdString(node_name));

        add_scene_rect(node_id, width, height, pose.value(), color, filename);
//qDebug()<<"Node"<<QString::fromStdString(node_name)<<"zvalue"<<pose.value().y();
    }
    else
    {
        qDebug()<<"Error gettion tranformation from node"<<QString::fromStdString(node_name)<<"to world";
    }
    
}

void DSRtoGraphicsceneViewer::add_scene_rect(std::int32_t node_id, std::int32_t width, std::int32_t height, QVec pose, std::string color, std::string texture)
{
    QBrush brush = QBrush(QColor(QString::fromStdString(color)));
    if (texture != "")
    {
        if(std::filesystem::exists(texture))
            brush = QBrush(QImage(QString::fromStdString(texture)));
        else
            brush = QBrush(QColor(QString::fromStdString(texture)));
    }
    QGraphicsRectItem * sceneRect;
    if (sceneMap.find(node_id) == sceneMap.end())
    {
//        sceneRect = scene.addRect(pose.x()-width/2 , pose.z() - height/2, width, height, QPen(QString::fromStdString(color)), brush);
        sceneRect = scene.addRect(0, 0, width, height, QPen(QString::fromStdString(color)), brush);
      //  sceneRect->setTransformOriginPoint(sceneRect->boundingRect().center());    

        sceneMap[node_id] = (QGraphicsItem*) sceneRect;
        create_parent_list(node_id);
    }
    else
    {
        sceneRect = (QGraphicsRectItem*) sceneMap[node_id];
    }
    sceneRect->setRect(pose.x()-width/2 , pose.z() - height/2, width, height);
    sceneRect->setPos(pose.x(), pose.z());
    sceneRect->setRotation(pose.ry()*180/M_PI);
    sceneRect->setZValue(pose.y());
    sceneRect->setBrush(brush);
}


void DSRtoGraphicsceneViewer::create_parent_list(std::int32_t node_id)
{

    auto node = G->get_node(node_id);
    int parent_id;
    int actual_id = node_id;
//std::cout<<"Node: "<<node_id<<" => ";
    do
    {
        parent_id = node.value().attrs()["parent"].value().dec();
        std::string edge_name = std::to_string(parent_id) + "_" + std::to_string(actual_id);
        edgeMap[edge_name].push_back(node_id);
        actual_id = parent_id;
        node = G->get_node(actual_id);

//std::cout<<edge_name<<" ";
    }while(node.value().type() != "world");

//std::cout<<std::endl;
    
}


void DSRtoGraphicsceneViewer::update_scene_rect_pose(std::int32_t node_id)
{
std::cout << "*************UPDATE NODE ******" << node_id<<std::endl;
    auto node = G->get_node(node_id);
    if (node.has_value())
    {
        std::optional<QVec> pose = innermodel->transformS6D("world", node.value().name());
        if (pose.has_value())
        {
    pose.value().print(QString::fromStdString(node.value().name()));

            auto item = sceneMap[node_id];
            item->setPos(pose.value().x(), pose.value().z());
            item->setRotation(pose.value().ry()*180/M_PI);
            item->setZValue(pose.value().y());
        }
    }
}
/*
void DSRtoGraphicsceneViewer::delete_scene_rect(std::int32_t node_id)
{

}

*/


//////////////////////////////////////////////////////////////
//                  MOUSE                                   //
//////////////////////////////////////////////////////////////
void DSRtoGraphicsceneViewer::wheelEvent(QWheelEvent* event)
{
//    qDebug()<<"wheel";
    const QGraphicsView::ViewportAnchor anchor = this->transformationAnchor();
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	qreal factor;
	if (event->angleDelta().y() > 0) 
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

void DSRtoGraphicsceneViewer::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        _pan = true;
        _panStartX = event->x();
        _panStartY = event->y();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        _pan = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (_pan)
    {
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - _panStartX));
        verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - _panStartY));
        _panStartX = event->x();
        _panStartY = event->y();
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::resizeEvent(QResizeEvent *e)
{  
//	qDebug() << "resize_graph_view" << x() << y()<<e->size(); 
	this->resize(e->size());
} 