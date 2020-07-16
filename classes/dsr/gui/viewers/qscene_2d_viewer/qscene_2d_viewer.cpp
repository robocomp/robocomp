#include "qscene_2d_viewer.h"

using namespace DSR ;

DSRtoGraphicsceneViewer::DSRtoGraphicsceneViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent) : AbstractGraphicViewer(parent)
{
    qDebug()<<"***************INIT DSRtoGraphicsceneViewer********************";
    G = G_;
    this->setMinimumSize(400,400);

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);

	this->scale(1, -1);


//REMOVE WHEN NOT NEEDED
    //center position
    scene.addRect(-100, -100, 200, 200, QPen(QColor("black")),QBrush(QColor("black")));
    //edge => x red, z (y) blue
    scene.addRect(-5000, -5000, 1000, 30, QPen(QColor("red")),QBrush(QColor("red")));
    scene.addRect(-5000, -5000, 30, 1000, QPen(QColor("blue")),QBrush(QColor("blue")));

    //initial graph
    create_graph();

    //update signals
    connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_node_slot);
	connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_edge_slot);

	connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &DSRtoGraphicsceneViewer::del_edge_slot);
	connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &DSRtoGraphicsceneViewer::del_node_slot);
}

void DSRtoGraphicsceneViewer::create_graph()
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
qDebug()<<"*************************";
qDebug() << __FUNCTION__ ;
    
    auto node = G->get_node(id);
    qDebug() << QString::fromStdString(node.value().name()) << " " << node.value().id() ;
    
    if(node.has_value())
    { 
        if (ignore_nodes.find(node.value().id()) != ignore_nodes.end())
            return;

        if( not check_RT_required_attributes(node.value()))
            return;

        if( type == "plane" )
            add_or_assign_plane(node.value());
        if( type == "mesh")
            add_or_assign_mesh(node.value());
        if( type == "person")
            add_or_assign_person(node.value());
        if( type == "differentialrobot" or type == "omnirobot")
            add_or_assign_robot(node.value());
        else //node does not have visual representation
            ignore_nodes.insert(id);

    }
}
void DSRtoGraphicsceneViewer::add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type)
{
qDebug()<<__FUNCTION__;
    //check if new edge connected any orphan nodes
    std::map<int, std::string>::iterator it = orphand_nodes.find(to);

    if(it != orphand_nodes.end())
    {
qDebug()<<"ORPHAND NODE FOUND"<<to;
        orphand_nodes.erase(it);
        add_or_assign_node_slot(it->first, it->second);
    }
    std::string edge_key = std::to_string(from) + "_" + std::to_string(to);
    for (int node_id : edge_map[edge_key])
    {
qDebug() << "******UPDATE EDGE "<<from << " " << to <<" update node " << node_id;
        update_scene_object_pose(node_id);
    }
}

//Compute 2d projection and zvalue
void DSRtoGraphicsceneViewer::get_2d_projection(std::string node_name, std::vector<int> size, QPolygon &polygon, int &zvalue)
{
    QVector<QPoint> polygon_vec;
    zvalue = -99999;
    //transform cube points
    std::optional<RTMat> rt = innermodel->getTransformationMatrixS("world", node_name);
    if (rt.has_value())
    {
        for (unsigned int i=0;i< cube_positions.size();i++)
        {
            QVec vec = rt.value() * QVec::vec4(size[0]*cube_positions[i][0], size[1]*cube_positions[i][1], size[2]*cube_positions[i][2], 1.0 );
            polygon_vec.append(QPoint(vec[0], vec[2]));
            if (zvalue < vec[1])
                zvalue = vec[1];
        }
    }
    else
    {
        qDebug()<<"Error gettting transformation matrix for node:"<<QString::fromStdString(node_name);   
    }
    polygon = QPolygon(polygon_vec);
} 


void DSRtoGraphicsceneViewer::add_or_assign_rect(Node &node, std::string color, std::string texture, int width, int height, int depth)
{

    //Calculate polygon
    QPolygon polygon;
    int zvalue;
    get_2d_projection(node.name(), {width, height, depth}, polygon, zvalue);
    QRect rect = QPolygon(polygon).boundingRect();

    //minimum size
    if(rect.width() < 100)
        rect.setWidth(100);
    if(rect.height() < 100)
        rect.setHeight(100);

    //texture
    QBrush brush = QBrush(QColor(QString::fromStdString(color)));
    if (texture != "")
    {
        if(std::filesystem::exists(texture))
            brush = QBrush(QImage(QString::fromStdString(texture)));
        else
            brush = QBrush(QColor(QString::fromStdString(texture)));
    }
   
    QGraphicsRectItem * sceneRect;
    if (scene_map.find(node.id()) == scene_map.end())
    {
        sceneRect = scene.addRect(rect, QPen(QString::fromStdString(color)), brush);
        scene_map[node.id()] = (QGraphicsItem*) sceneRect;
        std::list<int> parent_list = get_parent_list(node.id());
        update_edge_chain(parent_list);  
    }
    else
    {
        sceneRect = (QGraphicsRectItem*) scene_map[node.id()];
        sceneRect->setRect(rect);
    }
    sceneRect->setZValue(zvalue);

}


void DSRtoGraphicsceneViewer::add_or_assign_plane(Node &node)
{
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::list<int> parent_list = get_parent_list(node.id());
    //check if this node should be painted
    if(not is_drawable(parent_list))
    {
        ignore_nodes.insert(node.id());
        qDebug()<<"Node is not drawable => check parent list";
        return;
    }

    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    std::string texture = G->get_attrib_by_name<std::string>(node, "texture").value_or("");
    int width = G->get_attrib_by_name<std::int32_t>(node, "width").value_or(0);
    int height = G->get_attrib_by_name<std::int32_t>(node, "height").value_or(0);
    int depth = G->get_attrib_by_name<std::int32_t>(node, "depth").value_or(0);

    qDebug()<<"Draw plane"<<QString::fromStdString(node.name())<<"("<<width<<","<<height<<","<<depth<<")";
     
    add_or_assign_rect(node, color, texture, width, height, depth);
}

bool DSRtoGraphicsceneViewer::is_drawable(std::list<int> parent_list)
{
    bool drawable = true;
    for(std::list<int>::iterator it = parent_list.end(); it != parent_list.begin(); --it)
    {
        std::optional<Node> node = G->get_node(*it);
        if (node.has_value())
        { 
            std::string type = node.value().type();
            auto r = std::find(no_drawable_childs.begin(), no_drawable_childs.end(), type);
            if(r != no_drawable_childs.end())
            {
                drawable = false;
                break;
            }
        }
    }
    return drawable;
}

bool DSRtoGraphicsceneViewer::check_RT_required_attributes(Node node)
{
    try{
        std::optional<int> level = G->get_node_level(node);
        std::optional<int> parent = G->get_parent_id(node);
        std::optional<QVec> pose = innermodel->transformS6D("world", node.name());

        if(level.has_value() and parent.has_value() and pose.has_value())
            return true;
    }
    catch(...){    }
    orphand_nodes[node.id()] = node.type();
qDebug()<<"ORPHAN NODE"<<node.id()<<QString::fromStdString(node.type());
    return false;
}

void  DSRtoGraphicsceneViewer::add_or_assign_mesh(Node &node)
{   
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::list<int> parent_list = get_parent_list(node.id());
    //check if this node should be painted
    if(not is_drawable(parent_list))
    {
        ignore_nodes.insert(node.id());
        qDebug()<<"Node is not drawable => check parent list";
        return;
    }

    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    int width = G->get_attrib_by_name<std::int32_t>(node, "scalex").value_or(0);
    int height = G->get_attrib_by_name<std::int32_t>(node, "scaley").value_or(0);
    int depth = G->get_attrib_by_name<std::int32_t>(node, "scalez").value_or(0);

qDebug()<<"Draw mesh"<<QString::fromStdString(node.name())<<"("<<width<<","<<height<<","<<depth<<")"<<"color"<<QString::fromStdString(color);

    add_or_assign_rect(node, color, "", width, height, depth);

}

void  DSRtoGraphicsceneViewer::add_or_assign_person(Node &node)
{   
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::optional<QVec> pose;
    try
    {
        pose = innermodel->transformS6D("world", node.name());
    }catch(...){}
    if (pose.has_value())
    {
//pose.value().print(QString::fromStdString(node.name()));

        //check if person already exists
        QGraphicsPixmapItem * scenePixmap;
        if (scene_map.find(node.id()) == scene_map.end())
        {
            QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/dsr-graph/etc/person.png")).scaled(600,300);
            scenePixmap = scene.addPixmap(pixmap);
            scenePixmap->setZValue(pose.value().y());
            scene_map[node.id()] = (QGraphicsItem*) scenePixmap;
            std::list<int> parent_list = get_parent_list(node.id());
            update_edge_chain(parent_list);  
        }
        else
        {
            scenePixmap = (QGraphicsPixmapItem*) scene_map[node.id()];

        }
//qDebug()<<"angle"<<pose.value().ry()<<qRadiansToDegrees(pose.value().ry());        
        scenePixmap->setPos(pose.value().x() - 300, pose.value().z() - 150);
        scenePixmap->setRotation(qRadiansToDegrees(pose.value().ry())+180);    
    }
    else
    {
        qDebug()<<"Error getting transformation from person"<<QString::fromStdString(node.name())<<"to world";
    }
}

void DSRtoGraphicsceneViewer::add_or_assign_robot(Node &node)
{
    std::optional<QVec> pose;
    try
    {
        pose = innermodel->transformS6D("world", node.name());
    }catch(...){}
    if (pose.has_value())
    {
        //check if robot already exists
        QGraphicsPolygonItem * scenePolygon;
        if (scene_map.find(node.id()) == scene_map.end())
        {
            QPolygonF poly2;
            float size = ROBOT_LENGTH / 2.f;
            poly2 << QPoint(-size, -size)
                << QPoint(-size, size)
                << QPoint(-size / 3, size * 1.6)
                << QPoint(size / 3, size * 1.6)
                << QPoint(size, size)
                << QPoint(size, -size);
            QBrush brush;
            brush.setColor(QColor("DarkRed"));
            brush.setStyle(Qt::SolidPattern);
            scenePolygon = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush); 
            scenePolygon->setZValue(pose.value().y());
            scene_map[node.id()] = (QGraphicsItem*) scenePolygon;
            std::list<int> parent_list = get_parent_list(node.id());
            update_edge_chain(parent_list);  

        }
        else
        {
            scenePolygon = (QGraphicsPolygonItem*) scene_map[node.id()];      
        }
        scenePolygon->setPos(pose.value().x() - scenePolygon->boundingRect().center().x(), pose.value().z() - scenePolygon->boundingRect().center().y());
        scenePolygon->setRotation(qRadiansToDegrees(-pose.value().ry())+180);   
    }
    else
    {
        qDebug()<<"Error getting transformation from robot"<<QString::fromStdString(node.name())<<"to world";
    }
}

std::list<int> DSRtoGraphicsceneViewer::get_parent_list(std::int32_t node_id)
{
    std::list<int> parent_list;
    parent_list.push_back(node_id);
    auto node = G->get_node(node_id);
    try{
        std::optional<int> parent_id;
        do
        {
            parent_id = G->get_parent_id(node.value());
            if (parent_id.has_value())
            {
                parent_list.push_front(parent_id.value());
                node = G->get_node(parent_id.value());
            }
            else
                return std::list<int>();
        }while(node.value().type() != "world");
    }catch(...){}
    return parent_list;
}

//get all edges involve on node->world transformation chain
void DSRtoGraphicsceneViewer::update_edge_chain(std::list<int> parent_list)
{
    if (parent_list.size()< 2)
        return;
/*    qDebug()<<"PARENT_LIST: ";
    for(auto item : parent_list)
        qDebug()<<item<<" ";
    qDebug();*/
    std::list<int>::iterator first_id = parent_list.begin();
    std::list<int>::iterator second_id = parent_list.begin();
    second_id++;
    do
    {     
        std::string edge_name = std::to_string(*first_id) + "_" + std::to_string(*second_id);
//qDebug()<<edge_name<<" ";
        edge_map[edge_name].push_back(parent_list.back());
        
        first_id++;
        second_id++;
    }while( second_id != parent_list.end());
}


//update pose on edge changes
void DSRtoGraphicsceneViewer::update_scene_object_pose(std::int32_t node_id)
{
qDebug() << "*************UPDATE NODE ******" << node_id;
    auto node = G->get_node(node_id);
    if (node.has_value())
    {
        QGraphicsItem *item = scene_map[node_id];
        std::optional<QVec> pose;
        try
        {
            pose = innermodel->transformS6D("world", node.value().name());
//pose.value().print(QString::fromStdString(node.value().name()));
        }catch(...){};
        if (pose.has_value())
        {
            item->setPos(pose.value().x() - item->boundingRect().center().x(), pose.value().z() - item->boundingRect().center().y());
            item->setRotation(qRadiansToDegrees(-pose.value().ry())+180);
        }
        else   
        {
            qDebug()<<"Error gettion tranformation from person"<<QString::fromStdString(node.value().name())<<"to world";
        }

    }
}

void DSRtoGraphicsceneViewer::del_node_slot(const std::int32_t id)
{
qDebug() << "********************************";
qDebug() << __FUNCTION__ ; 
    //check if node has graphical representation
    if (scene_map.count(id) != 0)
    {
        QGraphicsItem *item = scene_map[id];
        delete item;
        scene_map.erase(id);
        //clear edge update chain
        for (std::map<std::string,std::vector<int>>::iterator it = edge_map.begin(); it != edge_map.end();it++)
	    {
            std::vector<int>::iterator pos = std::find(it->second.begin(), it->second.end(), id);
            if(pos != it->second.end())
                it->second.erase(pos);
        }
    }

    //remove from ignored just to keep consistency
    if (ignore_nodes.find(id) != ignore_nodes.end()) 
    {
        ignore_nodes.erase(id);
    }

    //TODO: check what happens with rt edges
}

void DSRtoGraphicsceneViewer::del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_tag)
{
    qDebug() << "********************************";
    qDebug() << __FUNCTION__ ;
    
    std::string edge_key = std::to_string(from) + "_" + std::to_string(to);
    qDebug() << "******Delete EDGE "<<QString::fromStdString(edge_key);
    if (edge_map.find(edge_key) != edge_map.end())
    {
        edge_map.erase(edge_key);
    }

//TODO: removed until signals detection workds properly

}

