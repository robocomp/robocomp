#include <osgviewer/osgview.h>

osg::Camera* OsgView::createHUD()
{
    // create a camera to set up the projection and model view matrices, and the subgraph to drawn in the HUD
    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,640,0,480));

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());


    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::PRE_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);

    // add to this camera a subgraph to render
    {

        osg::Geode* geode = new osg::Geode();

        // turn lighting off for the text and disable depth test to ensure its always ontop.
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        {
            osg::BoundingBox bb;
            osg::Geometry* geom = new osg::Geometry;

            osg::Vec3Array* vertices = new osg::Vec3Array;
            float depth = -10000000000.;

			bb.set(0,0,depth,640,480,depth);
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
            geom->setVertexArray(vertices);



            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            geom->setNormalArray(normals);
            geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->push_back(osg::Vec4(1.0f,1.0,1.0f,1.0f));
            geom->setColorArray(colors);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);

            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

			osg::Vec2Array* texcoords = new osg::Vec2Array(4);
			(*texcoords)[0].set(0.0f,0.0f);
			(*texcoords)[1].set(0.0f,1.0f);
			(*texcoords)[2].set(1.0f,1.0f);
			(*texcoords)[3].set(1.0f,0.0f);
			geom->setTexCoordArray(0,texcoords);

            osg::StateSet* stateset = geom->getOrCreateStateSet();
            stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
            //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
//             stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

            geode->addDrawable(geom);

			HUDTexture = new osg::Texture2D;

			// protect from being optimized away as static state:
			HUDTexture->setDataVariance(osg::Object::DYNAMIC);
			osgImage = new osg::Image();
			osgImage.get()->allocateImage(640,480, 1, GL_RGB, GL_UNSIGNED_BYTE, 1);

			// load an image by reading a file:
// 			osgImage = osgDB::readImageFile("a.jpg");



			// Assign the texture to the image we read from file:
			HUDTexture->setImage(osgImage);

			// Create a new StateSet with default settings:
			osg::StateSet* stateOne = new osg::StateSet();

			// Assign texture unit 0 of our new StateSet to the texture
			// we just created and enable the texture.
			stateOne->setTextureAttributeAndModes(0,HUDTexture,osg::StateAttribute::ON);
			// Associate this state set with the Geode that contains
			// the pyramid:
			geode->setStateSet(stateOne);
			stateOne = geode->getOrCreateStateSet();
			stateOne->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        }

        camera->addChild(geode);
    }

    return camera;
}

void OsgView::setImageHUD (osg::ref_ptr<osg::Image> i)
{
	HUDTexture->setImage(i);

}
osg::ref_ptr<osg::Image> OsgView::getImageHUD ( )
{
	return HUDTexture->getImage();
}

OsgView::OsgView(QWidget* parent)
{
     init(parent, false, NULL, 0);
}
OsgView::OsgView(QWidget* parent, bool hud, const QGLWidget* shareWidget, WindowFlags f): QGLWidget(parent, shareWidget, f) , osgViewer::Viewer()
{
     init(parent, hud, shareWidget, f);
}

void OsgView::init(QWidget* parent, bool hud, const QGLWidget* shareWidget, WindowFlags f)
{
	setFocusPolicy(Qt::StrongFocus);
	if (parent)
	{
		_gw = new osgViewer::GraphicsWindowEmbedded(0,0,parent->width(),parent->height());
		setParent( parent);
		resize(parent->width(), parent->height());
	}
	else
	{
		_gw = new osgViewer::GraphicsWindowEmbedded(0,0,800,800);
		resize(800,800);
	}
// 	osg::DisplaySettings::instance()->setNumMultiSamples( 4 );
	getCamera()->setViewport(new osg::Viewport(0, 0, width(), height()));
	getCamera()->setGraphicsContext(getGraphicsWindow());

	if (hud )
	{
		//matrix kinect
		getCamera()->setProjectionMatrixAsPerspective(58.7, 1., .00001, 10000.);
		getCamera()->setClearColor(osg::Vec4(1.,0.,0.,1.));
		getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
	// 	getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		setThreadingModel(osgViewer::Viewer::SingleThreaded);
// 		osg::DisplaySettings::instance()->setNumMultiSamples(4);
		initHUD();
		osgGA::TrackballManipulator *tBall = new osgGA::TrackballManipulator;
		///el eye en el cero, la camara en el 0 en X, 0 en Y, y a 4000 mirando al centro del eje de coordenadas, y arriba en sentido del eje Y+
		tBall->setHomePosition(osg::Vec3(0.,0.,0.),osg::Vec3(0.f,0.,-4.),osg::Vec3(0.0f,1.f,0.0f), false);
		setCameraManipulator( tBall );
	}
	else
	{
		getCamera()->setProjectionMatrixAsPerspective(55.0f, static_cast<double>(width())/static_cast<double>(height()), 0.000001, 100000.0);
		getCamera()->setClearColor(osg::Vec4(0.,0.,0.,1.));
 		setThreadingModel(osgViewer::Viewer::SingleThreaded);
// 		osg::DisplaySettings::instance()->setNumMultiSamples( 4 );
		init();
		osgGA::TrackballManipulator *tBall = new osgGA::TrackballManipulator;
		tBall->setHomePosition(osg::Vec3(0,0,0),osg::Vec3(0.f,0.,-40.),osg::Vec3(0.0f,1.f,0.0f), false);
		setCameraManipulator( tBall );
	}

	show();
}

void OsgView::setHomePosition(osg::Vec3 eye, osg::Vec3 center, osg::Vec3 up, bool autoComputeHomePosition)
{
	osgGA::TrackballManipulator *tBall = new osgGA::TrackballManipulator;
	tBall->setHomePosition (eye,center,up,autoComputeHomePosition);
	setCameraManipulator( tBall );
}

OsgView::~OsgView()
{
}

void OsgView::initHUD () {
//Build basic scene
	root = new osg::Group();
	osg::ref_ptr<osg::Group> group  = new osg::Group;

	group->addChild(root.get());
	TextGeode = new osg::Geode();
	textOne =  new osgText::Text();

	TextGeode->addDrawable(textOne);
	osg::Camera * c = createHUD();
	c->addChild(TextGeode);
 	group->addChild(c);
	setSceneData(group.get());

	connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
	timer.start(60);
}

void OsgView::init( )
{
	//Build basic scene
	root = new osg::Group();

 	//global stateset
	osg::StateSet *globalStateSet = new osg::StateSet;
	globalStateSet->setGlobalDefaults();
	globalStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

	// enable lighting
	globalStateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::Light* light = getLight();
	light->setAmbient(  osg::Vec4( 0.4f,    0.4f, 0.4f,  1.f ));
	light->setDiffuse(  osg::Vec4( 0.8f,    0.8f, 0.8f,  1.f ));
	light->setSpecular( osg::Vec4( 0.2f,    0.2f, 0.2f,  1.f ));
	light->setPosition( osg::Vec4( 0.0f, 3000.0f, 0.0f,  1.f));
// 	light->setDirection(osg::Vec3(0.0f, -1.0f, 0.0f));

	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
	lightSource->setLight(light);
	lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource->setStateSetModes(*globalStateSet,osg::StateAttribute::ON);
	root->addChild(lightSource.get() );

	addEventHandler(new osgViewer::WindowSizeHandler);

	setSceneData( root.get() ) ;

	connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
	timer.start(30);
}

void OsgView::paintGL()
{
	//This calls are commented to avoid auto-call to the OSG render process.
// 	frame();
	//printFPS();
}

osg::ShapeDrawable * OsgView::addRectangle(float px, float py, float pz, float w, float h, osg::PositionAttitudeTransform **pat)
{
	// box
	osg::Box *box = new osg::Box( osg::Vec3(px, py, pz), w, 0.03, h);
	// boxDrawable
	osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable( box ); //  boxDrawable HAS box
	boxDrawable->setColor(osg::Vec4(1.f,0.f,0.f,0.f));
	// shapeGeode
	osg::Geode* shapeGeode = new osg::Geode();                       //  shapeGeode HAS boxDrawable
	shapeGeode->addDrawable( boxDrawable );
	// pat
	(*pat) = new osg::PositionAttitudeTransform;
	(*pat)->addChild( shapeGeode );                                     //  pat HAS shapeGeode HAS boxDrawable HAS box
// 	(*pat)->setAttitude(osg::Quat(30, osg::Vec3(0.33333,0.33333,0.33333)));
	getRootGroup()->addChild(*pat);
	return boxDrawable;
}

osg::ShapeDrawable * OsgView::addBox(float px, float py, float pz, float sx, float sy, float sz, osg::PositionAttitudeTransform **pat, osg::Group *parentNode)
{
	// box
	osg::Box *box = new osg::Box( osg::Vec3(px, py, pz), sx, sy, sz);
	// boxDrawable
	osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable( box ); //  boxDrawable HAS box
	boxDrawable->setColor(osg::Vec4(1.f,0.f,0.f,0.f));
	// shapeGeode
	osg::Geode* shapeGeode = new osg::Geode();                       //  shapeGeode HAS boxDrawable
	shapeGeode->addDrawable( boxDrawable );
	// pat
	(*pat) = new osg::PositionAttitudeTransform;
	(*pat)->addChild( shapeGeode );                                     //  pat HAS shapeGeode HAS boxDrawable HAS box
	if (parentNode==NULL)
		getRootGroup()->addChild(*pat);
	else
		parentNode->addChild(*pat);

	return boxDrawable;
}

osg::ref_ptr<osg::Node> OsgView::addBall(const QVec &center, float radius, const QVec &color)
{
	osg::ref_ptr<osg::Sphere> ball = new osg::Sphere( qvecToVec3( center ), radius);
	osg::ref_ptr<osg::ShapeDrawable> sphereDrawable = new osg::ShapeDrawable( ball.get() );
	sphereDrawable->setColor( qvecToVec4(color));
	osg::ref_ptr<osg::Geode> shapeGeode = new osg::Geode();
	shapeGeode->addDrawable( sphereDrawable.get() );
	//getRootGroup()->addChild( shapeGeode.get() );
	return shapeGeode;

}

osg::ref_ptr<osg::Node> OsgView::addBall(const osg::Vec3 & center, float radius, const osg::Vec4 &color)
{
	osg::ref_ptr<osg::Sphere> ball = new osg::Sphere( center, radius);
	osg::ref_ptr<osg::ShapeDrawable> sphereDrawable = new osg::ShapeDrawable( ball.get() );
	sphereDrawable->setColor( color );
	osg::ref_ptr<osg::Geode> shapeGeode = new osg::Geode();
	shapeGeode->addDrawable( sphereDrawable.get() );
	//getRootGroup()->addChild( shapeGeode.get() );
	return shapeGeode.get();
}


void OsgView::removeBall(osg::Node * b)
{
	getRootGroup()->removeChild( b );
}

osg::ref_ptr<osg::Node> OsgView::addBox(const RMat::QVec &center, float side, const osg::Vec4 &color)
{
	osg::ref_ptr<osg::Box> box = new osg::Box( qvecToVec3( center ), side);
	osg::ref_ptr<osg::ShapeDrawable> boxDrawable = new osg::ShapeDrawable( box );
	boxDrawable->setColor( color );
	osg::ref_ptr<osg::Geode> shapeGeode = new osg::Geode();
	shapeGeode->addDrawable( boxDrawable.get() );
	return shapeGeode;

}

void OsgView::removeBox(osg::Node * b)
{
	getRootGroup()->removeChild( b );
}


/**
 * Computes a line and returns a osg::Shape
 * @param p1
 * @param p2
 * @param radius
 * @return
 */
osg::Shape * OsgView::addBasicLineShape(const QVec &p1, const QVec &p2, float radius)
{
	// compute line vector
	QVec l =  p2-p1;
	//l.print("l");
	float length = l.norm2();
	//qDebug() << length;
	if( length > 0.000001 )
	{
		// create a vertical vector parallel to he initial cylinder
		QVec ejeCil = QVec::vec3(0, 0, 1);
		// Cross product wih line vector to obtain axis of rotation
		QVec eje = l ^ ejeCil;
		// Compute angle o rotae with scalar product
		float ang = -acos((l * ejeCil) / length);
		//eje.print("eje");
		osg::Cylinder *line = new osg::Cylinder( qvecToVec3((p1+p2)/2.f), radius, length);
		line->setRotation(osg::Quat( ang, qvecToVec3(eje)));
		return line;
	}
	else
	{
		//qDebug() << "Warning: Could not add a zero length line";
		return NULL;
	}
}

/**
 * Computes a line and returns an osg::Node's compatible osg::Geode
 * @param p1
 * @param p2
 * @param radius
 * @param tip
 * @return
 */
osg::Node * OsgView::addBasicLine(const QVec &p1, const QVec &p2, float radius, bool tip, const osg::Vec4 & color)
{
	float length = (p2-p1).norm2();
	osg::Quat quat = quaternionFromInitFinalVector(QVec::vec3(0, 0, 1), p2-p1);
	if (length > 0.000001)
	{
		osg::Cylinder *line = new osg::Cylinder( qvecToVec3((p1+p2)/2.f), radius, length);
		line->setRotation(quat);
		osg::ShapeDrawable* lineDrawable = new osg::ShapeDrawable( line );
		lineDrawable->setColor(color);
		osg::Geode* shapeGeode = new osg::Geode();
		shapeGeode->addDrawable( lineDrawable );

		if (tip == true)
		{
			osg::Cone* tip = new osg::Cone(qmatToVec3(p2), 0.3f, 0.3);
			tip->setRotation(quat);
			osg::ShapeDrawable* tipDrawable = new osg::ShapeDrawable( tip );
			lineDrawable->setColor(color);
			shapeGeode->addDrawable( tipDrawable );
		}
		return shapeGeode;
	}
	else
	{
		qDebug() << "Warning: Cannot add a zero-length lines";
		return NULL;
	}
}

/**
 * Computes a line and returns a osg::Node as a child of getRootGroup()
 * @param p1
 * @param p2
 * @param radius
 * @param tip
 * @return
 */
osg::Node * OsgView::addLine(const RMat::QVec &p1, const RMat::QVec &p2, float radius, bool tip)
{
	osg::Node *linea = addBasicLine(p1, p2, radius, tip);
// 	getRootGroup()->addChild( linea );
	return linea;
}
/**
 * Computes an arrow and returns a osg::Node's as a child of getRootGroup()
 * @param p1
 * @param p2
 * @param radius
 * @return
 */
osg::Node * OsgView::addArrow(const RMat::QMat &p1, const RMat::QMat &p2, float radius)
{
	return addLine( p1, p2, radius, true);
}


osg::ref_ptr<osg::Node> OsgView::addPolyLine(const QVector< RMat::QVec > & pl, float radius)
{
	osg::ref_ptr<osg::CompositeShape> polyLine = new osg::CompositeShape;
	osg::ref_ptr<osg::Shape> line;
	//qDebug() << "In addPolyLine with " << pl.size() << " points";
	if( pl.size() > 1 )
	{
		for( int i = 0 ; i < pl.size() - 1 ; i ++ )
		{
			//qDebug() << pl.size() << pl[i](0) << pl[i](1) << pl[i](2) << pl[i+1](0) << pl[i+1](1) << pl[i+1](2);
			line = addBasicLineShape( pl[i] , pl[i+1] , radius );
			if( line != NULL )
				polyLine->addChild( line );
		}
		osg::ref_ptr<osg::ShapeDrawable> polyLineDrawable = new osg::ShapeDrawable( polyLine.get() );
		polyLineDrawable->setColor(osg::Vec4(0.7,0.7,0.,0.));
		osg::ref_ptr<osg::Geode> shapeGeode = new osg::Geode();
		shapeGeode->addDrawable( polyLineDrawable );
		//getRootGroup()->addChild( shapeGeode );
		return shapeGeode;
	}
	else return NULL;
}
///v vertices array
///color shape, RGB and Transparency
osg::ref_ptr<osg::Node> OsgView::addPolygon(const osg::Vec3Array &v, const osg::Vec4 &color)
{
	osg::ref_ptr<osg::Geometry> contourGeom = new osg::Geometry();

	contourGeom->setVertexArray((osg::Vec3Array *)&v);
	osg::ref_ptr<osg::DrawArrays> dr = new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, v.size());
	contourGeom->addPrimitiveSet(dr);

	osg::ref_ptr<osgUtil::Tessellator> tess = new osgUtil::Tessellator();

	tess->setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
	tess->setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
	tess->retessellatePolygons( *contourGeom );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(contourGeom);

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(color);
	contourGeom->setColorArray(colors);

	osg::StateSet *ss = contourGeom->getOrCreateStateSet();
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	contourGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
	contourGeom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );

	return geode;
}



void OsgView::addXYZAxisOnNode( osg::Group *node, float length, float radius, const osg::Vec4 & color )
{
	// X Axis
	QMat axis(3); axis.set(0.); axis(0) = length;
	osg::Node *lineaX = addBasicLine(QVec::zeros(3), axis, radius, true, color);
	//addArrow( QVec::vec3(0,0,0), axis, radius );
	osgText::Text *textX = new osgText::Text();
	textX->setText(osgText::String("X"));
	textX->setPosition(qmatToVec3(axis));
	textX->setCharacterSize(length);
	osg::Geode* shapeGeodeX = new osg::Geode();
	shapeGeodeX->addDrawable( textX );
	node->addChild(shapeGeodeX);
	node->addChild(lineaX);

	// Y Axis
	axis(0) = 0.; axis(1) = length;
	//addArrow( QVec::vec3(0,0,0), axis, radius );
	osg::Node *lineaY = addBasicLine(QVec::zeros(3), axis, radius, true, color);
	osgText::Text *textY = new osgText::Text();
	textY->setText( osgText::String("Y"));
	textY->setPosition( qmatToVec3(axis));
	textY->setCharacterSize(length);
	osg::Geode* shapeGeodeY = new osg::Geode();
	shapeGeodeY->addDrawable( textY );
	node->addChild(shapeGeodeY);
	node->addChild(lineaY);

	// Z Axis
	axis(1) = 0.; axis(2) = -length;
	//addArrow( QVec::vec3(0,0,0), axis, radius );
	osg::Node *lineaZ = addBasicLine(QVec::zeros(3), axis, radius, true, color);
	osgText::Text *textZ = new osgText::Text();
	textZ->setText( osgText::String("Z"));
	textZ->setPosition( qmatToVec3(axis));
	textZ->setCharacterSize(length);
	osg::Geode* shapeGeodeZ = new osg::Geode();
	shapeGeodeZ->addDrawable( textZ );
	node->addChild(shapeGeodeZ);
	node->addChild(lineaZ);
}


//Picking objects
/**
	* Finds the point of intersection of the mouse ray with the floor and obtains the coordinates.
*/

void OsgView::pickObject( const QPoint & p)
{

//     qDebug() << "Entering pickObject";
	// 0. Normalize mouse coordinates to -1,1 range.
	//For y, -1 would be the bottom of the window. 0.0 would be the middle of the window. +1 would be the top of the window.
	//For x, -1 would be the left hand side of the window. 0.0 would be the middle of the window. +1 would be the right hand side of the window.
	// 	QList<QPair<QPointF,QPointF> > listInt;
	// 	listInt.append(qMakePair( QPointF( 0.f, this->width() ), QPointF(-1.f,1.f)));
	// 	listInt.append(qMakePair( QPointF( 0.f, this->height() ), QPointF(-1.f, 1.f)));
	// 	QMat mt = QMat::afinTransformFromIntervals( listInt );
	// 	QVec pn = mt * QVec::homogeneousCoordinates(p);

  // 1. Create either a PolytopeIntersector, or a LineIntersector using the normalized mouse< co-ordinates
	osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, p.x(),-p.y() + height());

  // 2. Create an IntersectionVisitor, passing the Intersector as parameter to the constructor.
	osgUtil::IntersectionVisitor IntersectionVisitor(picker);

  // 3. Launch the IntersectionVisitor on the root node of the scene graph. In an OSGART application, we ca launch it on the osgART::Scene node.
	this->getCamera()->accept(IntersectionVisitor);

  // 4. If the Intersector contains any intersections obtain the NodePath and search it to find the Node of interest.
	 osg::Vec3d xc;
    if ( picker->containsIntersections())
	 {
		osg::NodePath nP = picker->getFirstIntersection().nodePath;

		// Here we search the NodePath for the node of interest. This is where we can make use of our node naming.
		//qDebug() << "Number of nodes to root " << nP.size();
		for (uint i = 0; i < nP.size(); i++)
		{
			{
			  xc = picker->getFirstIntersection().getWorldIntersectPoint();
			  QVec ret =  vec3ToQVec( xc );
				ret(2) = -ret(2);
			  emit newWorld3DCoor(ret);
			  break;
			}
		}
	  }
}

void OsgView::handle( const QPoint & p)
{
	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(this);
 if ( viewer )
 {
 osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
 new osgUtil::LineSegmentIntersector(
 osgUtil::Intersector::WINDOW, p.x(),-p.y() + height());

 osgUtil::IntersectionVisitor iv( intersector.get() );
 iv.setTraversalMask( ~0x1 );
 viewer->getCamera()->accept( iv );

 if ( intersector->containsIntersections() )
 {
 osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
 hexno = result.nodePath.back();
 kk = result.getWorldIntersectPoint();
 }

}
}

/// UTILITIES

/**
 * Converts column 3-vectors in RMat to osg::Vec3
 * @param m
 * @return
 */
osg::Vec3 OsgView::qmatToVec3(const RMat::QMat & m)
{
	return osg::Vec3(m(0),m(1),m(2));
}
osg::Vec4 OsgView::qvecToVec4(const RMat::QVec & m)
{
	return osg::Vec4(m(0),m(1),m(2),m(3));
}

RMat::QMat OsgView::vec3ToQMat(const osg::Vec3f & m)
{
	RMat::QMat r(3);
	r(0) = m.x();
	r(1) = m.y();
	r(2) = m.z();
	return r;
}

RMat::QVec OsgView::vec3ToQVec(const osg::Vec3f & m)
{
	RMat::QVec r(3);
	r(0) = m.x();
	r(1) = m.y();
	r(2) = m.z();
	return r;
}

RMat::QVec OsgView::vec4ToQVec(const osg::Vec4f & m)
{
	RMat::QVec r(4);
	r(0) = m.x();
	r(1) = m.y();
	r(2) = m.z();
	r(3) = m.w();
	return r;
}
/// EVENT HANDLING

void OsgView::printFPS( )
{
	static int fps=0;
	static QTime ti;
	if ((++fps % 50) == 0)
	{
		std::cout << "fps " << 50000 / ti.restart() << std::endl;
	}
}

void OsgView::resizeGL( int width, int height )
{
    _gw->getEventQueue()->windowResize(0, 0, width, height );
    _gw->resized(0,0,width,height);
}

void OsgView::keyPressEvent( QKeyEvent* event )
{
//	qDebug()<<"key pressed"<<event->text();
	if(event->key() == Qt::Key_Control)
	{
		flag1 = 1;
	}
	if(event->key() == Qt::Key_Q)
	{
		flag1 = 2;
		setCameraManipulator(0);
	}
	emit keyPress(event->text());
    _gw->getEventQueue()->keyPress( (osgGA::GUIEventAdapter::KeySymbol) *(event->text().toAscii().data() ) );
}

void OsgView::keyReleaseEvent( QKeyEvent* event )
{
	if(event->key() == Qt::Key_Control)
	{
		flag1 = 0;
	}
	if(event->key() == Qt::Key_Q)
	{
		flag1 = 0;
		osg::Vec3 eye, center, up; 
		this->getCamera()->getViewMatrixAsLookAt( eye, center, up ); 
		setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);

	}
	emit keyRelease(event->text());
    _gw->getEventQueue()->keyRelease( (osgGA::GUIEventAdapter::KeySymbol) *(event->text().toAscii().data() ) );
}

void OsgView::mousePressEvent( QMouseEvent* event )
{
    int button = 0;
    switch(event->button())
    {
        case(Qt::LeftButton): button = 1; {this->pickObject( QPoint(event->x(), event->y()) );flag2=1;} break;
        case(Qt::MidButton): button = 2; break;
        case(Qt::RightButton): button = 3;{this->handle( QPoint(event->x(), event->y()) ); flag2=2;}break;
        case(Qt::NoButton): button = 0; break;
        default: button = 0; break;
    }
    if (event->button() == Qt::LeftButton)
		emit newLeftCoor( event->posF() );
	else
		emit newRightCoor( event->posF() );
    _gw->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
}

void OsgView::mouseReleaseEvent( QMouseEvent* event )
{
    int button = 0;
    switch(event->button())
    {
        case(Qt::LeftButton): button = 1;{flag2=0; hexno=NULL;}break;
        case(Qt::MidButton): button = 2; break;
        case(Qt::RightButton): button = 3;{flag2=0; hexno = NULL;}break;
        case(Qt::NoButton): button = 0; break;
        default: button = 0; break;
    }
    emit endCoor(event->posF());
    _gw->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void OsgView::mouseMoveEvent( QMouseEvent* event )
{
    _gw->getEventQueue()->mouseMotion(event->x(), event->y());
    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(this);
 if ( viewer )
 {
 osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
 new osgUtil::LineSegmentIntersector(
 osgUtil::Intersector::WINDOW, event->x(),-event->y() + height());

 osgUtil::IntersectionVisitor iv( intersector.get() );
 iv.setTraversalMask( ~0x1 );
 viewer->getCamera()->accept( iv );

 if ( intersector->containsIntersections() )
 {
 osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
 kk = result.getWorldIntersectPoint();
 hexno = result.nodePath.back();

 }

}
}

osg::Quat OsgView::quaternionFromRotationMatrix(const QMat &rotM) const
{
	static const QVec initV = QVec::vec3(0, 1, 0);
	QVec destV = rotM*initV;
	QVec vQuat = destV^initV;
	const double aQuat = acos(initV.dotProduct(destV));

	if (vQuat.norm2() > 0.00000001)
	{
		return osg::Quat(-aQuat, osg::Vec3(vQuat(0), vQuat(1), vQuat(2)));
	}
	else
	{
		QVec initV = QVec::vec3(0, 0, 1);
		QVec destV = rotM*initV;
		const double aQuat = atan2(destV(2), destV(0));
		return osg::Quat(-aQuat+M_PIl/2., osg::Vec3(0, 1, 0));
	}
}

osg::Quat OsgView::quaternionFromInitFinalVector(const QVec &initV, const QVec &destV) const
{
	QVec vQuat = destV^initV;
	const double aQuat = acos(initV.dotProduct(destV));

	if (vQuat.norm2() > 0.00000001)
	{
		return osg::Quat(-aQuat, osg::Vec3(vQuat(0), vQuat(1), vQuat(2)));
	}
	else
	{
		const double aQuat = atan2(destV(2), destV(0));
		return osg::Quat(-aQuat+M_PIl/2., osg::Vec3(0, 1, 0));
	}
}
