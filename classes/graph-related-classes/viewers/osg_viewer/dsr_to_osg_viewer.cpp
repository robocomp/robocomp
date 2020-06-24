#include "dsr_to_osg_viewer.h"
#include <osg/Camera>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Material>
#include <osgGA/EventQueue>
#include <osgGA/TrackballManipulator>
#include <QMouseEvent>
 
using namespace DSR;

DSRtoOSGViewer::DSRtoOSGViewer(std::shared_ptr<CRDT::CRDTGraph> G_, float scaleX, float scaleY, QWidget *parent) : 
                        QOpenGLWidget(parent), 
                        _mGraphicsWindow(new osgViewer::GraphicsWindowEmbedded(this->x(), this->y(), this->width(), this->height())), 
                        _mViewer(new osgViewer::Viewer), m_scaleX(scaleX), m_scaleY(scaleY)
{
    G = G_;
    this->setMinimumSize(400, 400);
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setViewport( 0, 0, this->width(), this->height() );
    camera->setClearColor( osg::Vec4( 0.9f, 0.9f, 1.f, 1.f ) );
    float aspectRatio = this->width()/this->height();
    camera->setProjectionMatrixAsPerspective(55.0f, aspectRatio, 0.000001, 100000.0);
    camera->setGraphicsContext( _mGraphicsWindow );
    _mViewer->setCamera(camera);
	manipulator = new osgGA::TrackballManipulator;
//    manipulator->setAllowThrow( false );
//    this->setMouseTracking(true);
    osg::Vec3d eye(osg::Vec3(1000.,7000.,4000.));
    osg::Vec3d center(osg::Vec3(3000.,0., 0.));
    osg::Vec3d up(osg::Vec3(0.,1.,0.));
    manipulator->setHomePosition(eye, center, up, false);
    manipulator->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
    _mViewer->setCameraManipulator(manipulator);
    _mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
//    _mViewer->addEventHandler(new osgGA::GUIEventHandler());



	root = createGraph();

	analyse_osg_graph(root.get());
	qDebug()<< "End analyse";

	analyse_osg_graph(root.get());

	_mViewer->setSceneData(root.get());

    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, [this](auto id, auto type)
        { try
          {
            auto node = G->get_node(id);
            if (node.has_value())
                add_or_assign_node_slot(node.value());
          }
          catch(const std::exception &e){ std::cout << e.what() << std::endl; throw e;};
        });
	connect(G.get(), &CRDT::CRDTGraph::update_edge_signal,
        [this](auto from, auto to, auto type){
                                                auto parent = G->get_node(from);
                                                auto node = G->get_node(to);
                                                if(parent.has_value() and node.has_value())
                                                    add_or_assign_edge_slot(parent.value(), node.value());
                                                });
	//connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoOSGViewer::delEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoOSGViewer::delNodeSLOT);

    _mViewer->realize();
//    setMainCamera(manipulator, TOP_POV);
   
}

DSRtoOSGViewer::~DSRtoOSGViewer()
{
	root->removeChildren(0, root->getNumChildren());
	root->dirtyBound();
	root = NULL;
}

void DSRtoOSGViewer::initializeGL(){

    osg::ref_ptr<osg::StateSet> stateSet = root->getOrCreateStateSet();
    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

	// enable lighting
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	osg::Light* light = _mViewer->getLight();
	_mViewer->getLight()->setPosition(osg::Vec4(1,-1, 1, 0)); // make 4th coord 1 for point
	_mViewer->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
	_mViewer->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));

	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
	lightSource->setLight(light);
	lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource->setStateSetModes(*stateSet,osg::StateAttribute::ON);
	root->addChild(lightSource.get());
}

// We need to go down the tree breadth first
void DSRtoOSGViewer::traverse_RT_tree(const Node& node)
{
    for(auto &edge: G->get_edges_by_type(node, "RT"))
	{
        std::cout << __FUNCTION__ << " edges " << edge.from() << " " << edge.to() << " " << edge.type() << std::endl;
        auto child = G->get_node(edge.to());
        if(child.has_value())
        {
            add_or_assign_edge_slot(node, child.value());
            add_or_assign_node_slot(child.value());
            traverse_RT_tree(child.value());
        }
        else
            throw std::runtime_error("Unable to traverse the tree at node: " + std::to_string(edge.to()));
	}
}

osg::ref_ptr<osg::Group> DSRtoOSGViewer::createGraph()
{
    qDebug() << __FUNCTION__ << "Reading graph in OSG Viewer";
    try
    {
    	std::optional<Node> g_root = G->get_node_root().value();  //HAS TO BE TRANSFORM
        root = new osg::Group();
        osg_map.insert_or_assign(std::make_tuple(g_root.value().id(), g_root.value().id()), root);

        traverse_RT_tree(g_root.value());
        qDebug() << __FUNCTION__ << "Finished reading graph. Waiting for events";
        return root;
    }
	catch(const std::exception &e) { std::cout << e.what() << " Error in method "<< __FUNCTION__<<" : " << std::endl;}
    return osg::ref_ptr<osg::Group>();
}

///////////////////////////////////////////////////////////////////////////////////


void  DSRtoOSGViewer::setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov) const
{
	osg::Quat mRot;

	switch(pov)
	{
        case TOP_POV:
            mRot.makeRotate(-M_PI_2, QVecToOSGVec(QVec::vec3(1,0,0)));
            break;
        case BACK_POV:
            mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,0,0)));
            break;
        case FRONT_POV:
            mRot.makeRotate(M_PI,    QVecToOSGVec(QVec::vec3(0,1,0)));
            break;
        case LEFT_POV:
            mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,-1,0)));
            break;
        case RIGHT_POV:
            mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,1,0)));
            break;
        default:
            qFatal("InnerModelViewer: invalid POV.");
	}
	manipulator->setRotation(mRot);
}


//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////

// To insert a node its parent has to be a transform
// Transforms are created form RT edges.

void DSRtoOSGViewer::add_or_assign_node_slot(const Node &node)
{
    std::cout << __FUNCTION__  << " node " << node.id() << " " << std::endl;
    try
    {
        if(node.id() == G->get_node_root().value().id())
            return;
        auto parent_id = G->get_parent_id(node);
        if (not parent_id.has_value())
            throw std::runtime_error("Node cannot be inserted without a parent");
        auto parent = G->get_node(parent_id.value());
        if (not parent.has_value())
            throw std::runtime_error("Non existing node: " + std::to_string(parent_id.value()));
        auto type = node.type();
        std::cout << "      " << node.name() << " " << node.id() << " " << node.type() << std::endl;
        if( type == "plane" )
            add_or_assign_box(node, parent.value());
        else if( type == "mesh")
            add_or_assign_mesh(node, parent.value());
        else if( type == "transform")
            add_or_assign_transform(node, parent.value());
        else if( type == "differentialRobot" or type == "laser" or type == "rgbd" or type == "omniRobot")
            add_or_assign_transform(node, parent.value());
    }
    catch(const std::exception& e)
    {  
        std::cout << "Exception in " << __FUNCTION__ << " " << e.what() << std::endl; 
        throw e;  
    }
}

void DSRtoOSGViewer::add_or_assign_edge_slot(const Node &from, const Node& to)
{
    std::cout << __FUNCTION__ << "from " << from.id() << " to " << to.id() << std::endl;
    auto edge = G->get_edge(from.id(), to.id(), "RT");
    if (edge.has_value())
    {
        //auto edge = G->get_edge_RT(from, to.id());
        auto rtmat = G->get_edge_RT_as_RTMat(edge.value());
        rtmat.print("rtmat");
        auto mat = QMatToOSGMat4(rtmat);

        // Insert
        if (auto res = osg_map.find(std::make_tuple(from.id(), from.id())); res != osg_map.end()) {
            if (auto anterior = osg_map.find(std::make_tuple(from.id(), to.id())); anterior != osg_map.end()) {
                if (auto old_trans = dynamic_cast<osg::MatrixTransform *>((*anterior).second); old_trans != nullptr)
                    old_trans->setMatrix(mat);
                else
                    throw std::runtime_error("Exception: dynamic_cast to MatrixTransform failed");
            } else {
                osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform(mat);
                (*res).second->addChild(transform);
                osg_map.insert_or_assign(std::make_tuple(from.id(), to.id()), transform);
            }
            qDebug() << __FUNCTION__ << "Added transform, node " << to.id() << "parent " << from.id();
        } else
            throw std::runtime_error("Exception: parent " + from.name() + " not found for node " + to.name());
    }
}

////////////////////////////////////////////////////////////////
/// Node modifications
////////////////////////////////////////////////////////////////

void DSRtoOSGViewer::add_or_assign_transform(const Node &node, const Node& parent)
{
    std::cout << __FUNCTION__  << "node " << node.id() << "parent "  << parent.id() << std::endl;
   
    if( auto res = osg_map.find(std::make_tuple(parent.id(), node.id())); res != osg_map.end())   
    {
        if( auto anterior = osg_map.find(std::make_tuple(node.id(), node.id())); anterior == osg_map.end())
        {
            osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform();
            transform->setName(std::to_string(node.id())+"-"+std::to_string(node.id()));
            (*res).second->addChild(transform);
            osg_map.insert_or_assign(std::make_tuple(node.id(), node.id()), transform);
        }
        qDebug() << __FUNCTION__ << "Added transform, node " << node.id() << "parent "  << parent.id();
    }
    else
        throw std::runtime_error("Transform: OSG parent not found for " + parent.name() + "-" +std::to_string(parent.id()));
}

void DSRtoOSGViewer::add_or_assign_box(const Node &node, const Node& parent)
{
    std::cout << __FUNCTION__ << ": node " <<  node.name() << "-" << node.id()<< " Parent: " << parent.id() << std::endl;
    try
    {
        auto texture = G->get_attrib_by_name<std::string>(node, "texture");
        if(texture.has_value()) std::cout << texture.value() << std::endl;
        auto height = G->get_attrib_by_name<std::int32_t>(node, "height");
        if(height.has_value()) std::cout << height.value() << std::endl;
        auto width = G->get_attrib_by_name<std::int32_t>(node, "width");
        if(width.has_value()) std::cout << height.value() << std::endl;
        auto depth = G->get_attrib_by_name<std::int32_t>(node, "depth");
        if(depth.has_value()) std::cout << depth.value() << std::endl;
   
        // Check valid ranges
        auto textu = texture.value_or("#000000");
        bool constantColor = false;
        if (textu.size() == 7 and textu[0] == '#')
                constantColor = true;

        // Create object
        osg::ref_ptr<osg::Box> box = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), width.value(), height.value(), depth.value());
		osg::ref_ptr<osg::ShapeDrawable> plane_drawable = new osg::ShapeDrawable(box);
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(plane_drawable);
		osg::ref_ptr<osg::Group> group = new osg::Group;
        group->setName(std::to_string(node.id())+"-"+std::to_string(node.id()));
        group->addChild(geode);
        
        // Insert
        if( auto res = osg_map.find(std::make_tuple(parent.id(), node.id())); res != osg_map.end())   
        {
            if( auto anterior = osg_map.find(std::make_tuple(node.id(), node.id())); anterior == osg_map.end())
            {
                (*res).second->addChild(group);
                osg_map.insert_or_assign(std::make_tuple(node.id(), node.id()), group);
                qDebug() << __FUNCTION__ << "Added transform, node " << node.id() << "parent "  << parent.id();
            }
            // Already exists. Just modify it
        }
        else
            throw std::runtime_error("Transform: OSG parent not found for " + node.name() + "-" +std::to_string(node.id()));

        // All these depending on new or mod
        // add texture 
        if (constantColor)
            plane_drawable->setColor(htmlStringToOsgVec4(textu));
        else
        {
            // image
			  osg::ref_ptr<osg::Image> image;
            if (textu.size()>0 and not constantColor)
                if( image = osgDB::readImageFile(textu), image == nullptr)
                    throw std::runtime_error("Couldn't load texture from file: " + texture.value());
            // texture
            osg::ref_ptr<osg::Texture2D>texture = new osg::Texture2D;
            texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
            texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
            texture->setImage(image);
            //texture->setDataVariance(Object::DYNAMIC);
            texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
            texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
            texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
            texture->setTextureWidth(1);
            texture->setTextureHeight(1);
            texture->setResizeNonPowerOfTwoHint(false);
            // Material
            osg::ref_ptr<osg::Material> material = new osg::Material();
            //material->setTransparency( osg::Material::FRONT_AND_BACK, 0);
            material->setEmission(osg::Material::FRONT, osg::Vec4(0.8, 0.8, 0.8, 0.5));
            // Assign the material and texture to the plane
            osg::ref_ptr<osg::StateSet> sphereStateSet = geode->getOrCreateStateSet();
            sphereStateSet->ref();
            sphereStateSet->setAttribute(material);
            sphereStateSet->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
            sphereStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        }
     }
    catch(const std::exception &e)
    { std::cout << "Exception at insert_or_assign_box " << e.what() <<  std::endl; throw e; }
}

void  DSRtoOSGViewer::add_or_assign_mesh(const Node &node, const Node& parent)
{   
    std::cout << __FUNCTION__ << "node " << node.id() << parent.id() << std::endl;
    auto color = G->get_attrib_by_name<std::string>(node, "color");
    if(color.has_value()) std::cout << color.value() << std::endl;
    auto filename = G->get_attrib_by_name<std::string>(node, "path");
    if(filename.has_value()) std::cout << filename.value() << std::endl;
    auto scalex = G->get_attrib_by_name<std::int32_t>(node, "scalex");
    if(scalex.has_value()) std::cout << scalex.value() << std::endl;
    auto scaley = G->get_attrib_by_name<std::int32_t>(node, "scaley");
    if(scaley.has_value()) std::cout << scaley.value() << std::endl;
    auto scalez = G->get_attrib_by_name<std::int32_t>(node, "scalez");
    if(scalez.has_value()) std::cout << scalez.value() << std::endl;
    
    // Check valid ranges
    
    // Create object
    osg::ref_ptr<osg::MatrixTransform> scale_transform = new osg::MatrixTransform; 			
	scale_transform->setMatrix(osg::Matrix::scale(scalex.value(), scaley.value(), scalez.value()));
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    osg::ref_ptr<osg::Node> osg_mesh = osgDB::readNodeFile(filename.value());
    if (!osg_mesh)
        throw  std::runtime_error("Could not find nesh file " + filename.value());
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode();
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
    osg_mesh->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    osg_mesh->getOrCreateStateSet()->setMode( GL_RESCALE_NORMAL, osg::StateAttribute::ON );
    scale_transform->addChild(osg_mesh);
    scale_transform->setName(std::to_string(node.id())+"-"+std::to_string(node.id()));
            
    // Insert
    if( auto res = osg_map.find(std::make_tuple(parent.id(), node.id())); res != osg_map.end())   
    {
        if( auto anterior = osg_map.find(std::make_tuple(node.id(), node.id())); anterior == osg_map.end())
        {
            (*res).second->addChild(scale_transform);
            osg_map.insert_or_assign(std::make_tuple(node.id(), node.id()), scale_transform);
            qDebug() << __FUNCTION__ << "Added transform, node " << node.id() << "parent "  << parent.id();
        }
        // Already exists. Just modify it
    }
    else
        throw std::runtime_error("Transform: OSG parent not found for " + node.name() + "-" +std::to_string(node.id()));

}

/////////////////////////////////////////////////////////////
//////// Auxiliary methods
/////////////////////////////////////////////////////////////

osg::Vec3 DSRtoOSGViewer::QVecToOSGVec(const QVec &vec) const 
{
	return osg::Vec3(vec(0), vec(1), -vec(2));
}

osg::Vec4 DSRtoOSGViewer::htmlStringToOsgVec4(const std::string &color)
{
	QString red   = QString("00");
	QString green = QString("00");
	QString blue  = QString("00");
	bool ok;
	red[0]   = color[1]; red[1]   = color[2];
	green[0] = color[3]; green[1] = color[4];
	blue[0]  = color[5]; blue[1]  = color[6];
	return osg::Vec4(float(red.toInt(&ok, 16))/255., float(green.toInt(&ok, 16))/255., float(blue.toInt(&ok, 16))/255., 0.f);
}

osg::Matrix  DSRtoOSGViewer::QMatToOSGMat4(const RTMat &nodeB)
{
	QVec angles = nodeB.extractAnglesR();
	QVec t = nodeB.getTr();
	RTMat node = RTMat(-angles(0), -angles(1), angles(2), QVec::vec3(t(0), t(1), -t(2)));

	return osg::Matrixd( node(0,0), node(1,0), node(2,0), node(3,0),
	                     node(0,1), node(1,1), node(2,1), node(3,1),
	                     node(0,2), node(1,2), node(2,2), node(3,2),
	                     node(0,3), node(1,3), node(2,3), node(3,3) );
}

////////////////////////////////////////////////////////////////
/////  Painting events
///////////////////////////////////////////////////////////////

inline void DSRtoOSGViewer::paintGL() 
{
    _mViewer->frame();
}

void DSRtoOSGViewer::resizeGL( int width, int height ) 
{
    this->getEventQueue()->windowResize(this->x()*m_scaleX, this->y() * m_scaleY, width*m_scaleX, height*m_scaleY);
    _mGraphicsWindow->resized(this->x()*m_scaleX, this->y() * m_scaleY, width*m_scaleX, height*m_scaleY);
    osg::Camera* camera = _mViewer->getCamera();
    float aspectRatio = this->width() / this->height();
    camera->setProjectionMatrixAsPerspective(55.0f, aspectRatio, 0.000001, 100000.0);
    camera->setViewport(0, 0, this->width()*m_scaleX, this->height()* m_scaleY);
}


void DSRtoOSGViewer::resizeEvent(QResizeEvent *e)
{  
//    qDebug() << "OSG => SCALE" << x() << y(); 
    this->resize(e->size());
} 

void DSRtoOSGViewer::mouseMoveEvent(QMouseEvent* event)
{
    this->getEventQueue()->mouseMotion(event->x()*m_scaleX, event->y()*m_scaleY);
}

void DSRtoOSGViewer::mousePressEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    this->getEventQueue()->mouseButtonPress(event->x()*m_scaleX, event->y()*m_scaleY, button);
}

void DSRtoOSGViewer::mouseReleaseEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    this->getEventQueue()->mouseButtonRelease(event->x()*m_scaleX, event->y()*m_scaleY, button);
}

void DSRtoOSGViewer::wheelEvent(QWheelEvent* event)
{
    int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?
                osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
    this->getEventQueue()->mouseScroll(motion);
}

bool DSRtoOSGViewer::event(QEvent* event)
{
    bool handled = QOpenGLWidget::event(event);
    this->update();
    return handled;
}

osgGA::EventQueue* DSRtoOSGViewer::getEventQueue() const 
{
    osgGA::EventQueue* eventQueue = _mGraphicsWindow->getEventQueue();
    return eventQueue;
}

//////////////////////////////////////////////////////////////////////////////

void DSRtoOSGViewer::analyse_osg_graph(osg::Node *nd) 
{
	/// here you have found a group.
    osg::Geode *geode = dynamic_cast<osg::Geode *> (nd);
	if (geode) 
    { // analyse the geode. If it isnt a geode the dynamic cast gives NULL.
        osg::notify(osg::WARN) << " Geode "<<  geode->getName() <<std::endl;
	    for (unsigned int i=0; i<geode->getNumDrawables(); i++) 
        {
		    // osg::Drawable *drawable=geode->getDrawable(i);
		    // osg::Geometry *geom=dynamic_cast<osg::Geometry *> (drawable);
		    // for (unsigned int ipr=0; ipr<geom->getNumPrimitiveSets(); ipr++) 
            // {
			//     osg::PrimitiveSet* prset=geom->getPrimitiveSet(ipr);
			//     osg::notify(osg::WARN) << "Primitive Set "<< ipr << std::endl;
			//     //analysePrimSet(prset, dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray()));
		    // }
	    } 
    }
    else 
    {
		osg::Group *gp = dynamic_cast<osg::Group *> (nd);
		if (gp) 
        {
			osg::notify(osg::WARN) << "Group "<<  gp->getName() <<std::endl;
			for (unsigned int ic=0; ic<gp->getNumChildren(); ic++) {
				analyse_osg_graph(gp->getChild(ic));
			}
		} else 
        {
			osg::notify(osg::WARN) << "Unknown node "<<  nd <<std::endl;
		}
	}
}

// void OsgView::keyReleaseEvent( QKeyEvent* event )
// {
// 	if(event->key() == Qt::Key_Control)
// 	{
// 		flag1 = 0;
// 	}
// 	if(event->key() == Qt::Key_Q)
// 	{
// 		flag1 = 0;
// 		osg::Vec3 eye, center, up; 
// 		this->getCamera()->getViewMatrixAsLookAt( eye, center, up ); 
// 		setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);

// 	}
// 	emit keyRelease(event->text());
//     _gw->getEventQueue()->keyRelease( (osgGA::GUIEventAdapter::KeySymbol) *(event->text().toLatin1().data() ) );
// }
