/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>




// ------------------------------------------------------------------------------------------------
// InnerModelViewer
// ------------------------------------------------------------------------------------------------
InnerModelViewer::InnerModelViewer(const std::shared_ptr<InnerModel> &im, QString root,  osg::Group *parent, bool ignoreCameras) : osg::Switch()
{	
	//Copy innermodel from param
	innerModel = im;
	
	if( innerModel.get() == nullptr )
		throw "InnerModelViewer::InnerModelViewer(): Error, InnerModel is nullptr";
	
	// Get main node
	InnerModelNode *imnode = innerModel->getNode(root);
	if (not imnode)
	{
		qDebug() << "InnerModelViewer::InnerModelViewer(): Error: Specified root node" << root << "not found.";
		throw "InnerModelViewer::InnerModelViewer(): Error: Specified root node not found.";
	}
	recursiveConstructor(imnode, this, mts, meshHash, ignoreCameras); //mts, osgmeshes, osgmeshPats);
	
	// Update
	update();
	if (parent)
		parent->addChild(this);
}

// ------------------------------------------------------------------------------------------------
// InnerModelViewer
// ------------------------------------------------------------------------------------------------
InnerModelViewer::InnerModelViewer(InnerModel *im, QString root,  osg::Group *parent, bool ignoreCameras) : osg::Switch()
{	
	// Initialize InnerModel from innermodel pointer
	innerModel = std::make_shared<InnerModel>(im);
	
	if( innerModel.get() == nullptr )
		throw "InnerModelViewer::InnerModelViewer(): Error, InnerModel is nullptr";
	
	// Get main node
	InnerModelNode *imnode = innerModel->getNode(root);
	if (not imnode)
	{
		qDebug() << "InnerModelViewer::InnerModelViewer(): Error: Specified root node" << root << "not found.";
		throw "InnerModelViewer::InnerModelViewer(): Error: Specified root node not found.";
	}
	recursiveConstructor(imnode, this, mts, meshHash, ignoreCameras); //mts, osgmeshes, osgmeshPats);
	
	// Update
	update();
	if (parent)
		parent->addChild(this);
}

InnerModelViewer::~InnerModelViewer()
{
// 	delete innerModel;
}

void InnerModelViewer::recursiveConstructor(InnerModelNode *node)
{
	osg::Group* parent;
	if (not node->parent)
		parent = this; 
	else
		parent = mts[node->parent->id];	
	recursiveConstructor(node, parent, mts, meshHash);	
}

void InnerModelViewer::recursiveRemove(InnerModelNode *node)
{
	InnerModelTransform *transformation;
	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelLaser *laser;
	InnerModelPointCloud *pointcloud;
	InnerModelDisplay *display;
	
	// Find out which kind of node are we dealing with
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		mts.remove(transformation->id); 
		for(int i=0; i<node->children.size(); i++)
		{
			recursiveRemove(node->children[i]);
		}
	}
	else if ((laser = dynamic_cast<InnerModelLaser *>(node)))
	{
		lasers.remove(node->id);
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		while(planesHash[node->id]->getNumParents() > 0)
			((osg::Group *)(planesHash[node->id]->getParent(0)))->removeChild(planesHash[node->id]);
		planeMts.remove(plane->id);
		planesHash.remove(node->id);
	}
	else if ((display = dynamic_cast<InnerModelDisplay *>(node)))
	{
		while(planesHash[node->id]->getNumParents() > 0)
			((osg::Group *)(planesHash[node->id]->getParent(0)))->removeChild(planesHash[node->id]);
		planeMts.remove(node->id);
		planesHash.remove(node->id);
	}
	else if ((pointcloud = dynamic_cast<InnerModelPointCloud *>(node)))
	{
		pointCloudsHash.remove(node->id);
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		while (meshHash[node->id].osgmeshPaths->getNumParents() > 0)
			meshHash[node->id].osgmeshPaths->getParent(0)->removeChild(meshHash[node->id].osgmeshPaths);
		while(meshHash[node->id].osgmeshes->getNumParents() > 0)
			meshHash[node->id].osgmeshes->getParent(0)->removeChild(meshHash[node->id].osgmeshes);
		while(meshHash[node->id].meshMts->getNumParents() > 0)
			meshHash[node->id].meshMts->getParent(0)->removeChild(meshHash[node->id].meshMts);
		meshHash.remove(mesh->id);
	}
}

//CAUTION
void InnerModelViewer::recursiveConstructor(InnerModelNode *node, osg::Group* parent,QHash<QString, osg::MatrixTransform *> &mtsHash, QHash<QString, IMVMesh> &meshHash, bool ignoreCameras)
{
	InnerModelTouchSensor *touch;
	InnerModelMesh *mesh;
	InnerModelPointCloud *pointcloud;
	InnerModelPlane *plane;
	InnerModelCamera *camera;
	InnerModelRGBD *rgbd;
	InnerModelIMU *imu;
	InnerModelLaser *laser;
	InnerModelTransform *transformation;
	InnerModelDisplay *display;

	// Find out which kind of node are we dealing with
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		// Create and include MT
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;

		if (parent) parent->addChild(mt);
		
 		mtsHash[transformation->id] = mt;
		
		mt->setMatrix(QMatToOSGMat4(*transformation));
		
		for(int i=0; i<node->children.size(); i++)
		{
			recursiveConstructor(node->children[i], mt, mtsHash, meshHash, ignoreCameras);
		}
	}
	else if ((rgbd = dynamic_cast<InnerModelRGBD *>(node)))
	{
		if ((not ignoreCameras) and rgbd->port)
		{
			IMVCamera cam;
			// Camera ID
			cam.id = node->id;
			// XML node for the camera
			cam.RGBDNode = rgbd;
			
			// Viewer
			cam.viewerCamera = new osgViewer::Viewer();
		
			double fov = 2. * atan2(0.5*cam.RGBDNode->height, cam.RGBDNode->focal);
			double aspectRatio = cam.RGBDNode->width / cam.RGBDNode->height;
			double zNear = 0.01, zFar = 10000.0;
			
			// Images
			cam.rgb = new osg::Image;
			cam.rgb->allocateImage(rgbd->width, rgbd->height, 1, GL_RGB, GL_UNSIGNED_BYTE);
			cam.d = new osg::Image;
			cam.d->allocateImage(rgbd->width, rgbd->height, 1, GL_DEPTH_COMPONENT,GL_FLOAT);
			cam.manipulator = new osgGA::TrackballManipulator();
			// CAUTION check y cambiar tb camera
			cam.viewerCamera->setSceneData(this);
			cam.viewerCamera->setUpViewInWindow( 0, 0, rgbd->width, rgbd->height);
			cam.viewerCamera->getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			cam.viewerCamera->getCamera()->attach(osg::Camera::COLOR_BUFFER, cam.rgb);
			cam.viewerCamera->getCamera()->attach(osg::Camera::DEPTH_BUFFER, cam.d);
			cam.viewerCamera->getCamera()->setProjectionMatrix(::osg::Matrix::perspective(fov*180./M_PIl, aspectRatio, zNear, zFar));
			
			// set windowName to innerModel id. Using Traits!
			osg::ref_ptr< osg::GraphicsContext::Traits > traits =  new osg::GraphicsContext::Traits (*cam.viewerCamera->getCamera()->getGraphicsContext()->getTraits());
			traits->windowName = cam.id.toStdString();
			traits->supportsResize = false;
			cam.viewerCamera->getCamera()->setGraphicsContext(osg::GraphicsContext::createGraphicsContext( traits.get() ));
			
			RTMat rt = innerModel->getTransformationMatrix("root", cam.id);
			cam.viewerCamera->setCameraManipulator(cam.manipulator);
			cam.viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));
			cameras[cam.id] = cam;
		}
	}
	else if ((camera = dynamic_cast<InnerModelCamera *>(node)))
	{
		if (not ignoreCameras)
		{
		}
	}
	else if ((imu = dynamic_cast<InnerModelIMU *>(node)))
	{
	}
	else if ((laser = dynamic_cast<InnerModelLaser *>(node)))
	{
		IMVLaser iml;
		iml.id = node->id;
		iml.osgNode = new osg::Switch();
		iml.laserNode = laser;
		parent->addChild(iml.osgNode);
		lasers[iml.id] = iml;
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		// Create plane's specific mt
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		planeMts[plane->id] = mt;
		IMVPlane *imvplane = new IMVPlane(plane, plane->texture.toStdString(), osg::Vec4(0.8,0.5,0.5,0.5), 0); 
		planesHash[node->id] = imvplane;
		setOSGMatrixTransformForPlane(mt, plane);
		if (parent) parent->addChild(mt);
		mt->addChild(imvplane);
	}
	else if ((display = dynamic_cast<InnerModelDisplay *>(node)))
	{
		// Create plane's specific mt
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		planeMts[display->id] = mt;
		IMVPlane *imvplane = new IMVPlane(display, display->texture.toStdString(), osg::Vec4(0.8,0.5,0.5,0.5), 0);
		planesHash[node->id] = imvplane;
		setOSGMatrixTransformForDisplay(mt, display);
		if (parent) parent->addChild(mt);
		mt->addChild(imvplane);
	}
	else if ((pointcloud = dynamic_cast<InnerModelPointCloud *>(node)))
	{
		IMVPointCloud *imvpc = new IMVPointCloud(node->id.toStdString());
		pointCloudsHash[node->id] = imvpc;
		parent->addChild(imvpc);
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		// Create mesh's specific mt
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		
		if (parent) parent->addChild(mt);
		
		RTMat rtmat = RTMat();
		rtmat.setR (mesh->rx, mesh->ry, mesh->rz);
		rtmat.setTr(mesh->tx, mesh->ty, mesh->tz);
		mt->setMatrix(QMatToOSGMat4(rtmat));
		
		osg::ref_ptr<osg::MatrixTransform> smt = new osg::MatrixTransform; 		
		
		smt->setMatrix(osg::Matrix::scale(mesh->scalex,mesh->scaley,mesh->scalez));
		mt->addChild(smt);
		meshHash[mesh->id].osgmeshPaths = mt;

		// Create mesh
		osg::ref_ptr<osg::Node> osgMesh = osgDB::readNodeFile(mesh->meshPath.toStdString());
		if (!osgMesh)
			printf("Could not find %s osg.\n", mesh->meshPath.toStdString().c_str());
		
		osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode();
		if (mesh->render == InnerModelMesh::WireframeRendering) // wireframe
			polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
		else
			polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);

		osgMesh->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		osgMesh->getOrCreateStateSet()->setMode( GL_RESCALE_NORMAL, osg::StateAttribute::ON );
		meshHash[mesh->id].osgmeshes = osgMesh;
		meshHash[mesh->id].meshMts= mt;
		osgmeshmodes[mesh->id] = polygonMode;
		smt->addChild(osgMesh);
	}
	else if ((touch = dynamic_cast<InnerModelTouchSensor *>(node)))
	{
	}
	else
	{
		qDebug() << "InnerModelReader::InnerModelReader(): Error: Unknown type of node";
		throw "InnerModelReader::InnerModelReader(): Error: Unknown type of node";
	}
// 	qDebug()<<"size mts mtsHash "<<mts.size()<<mtsHash.size();
// 	qDebug()<<"end of recursiveConstructor mtsHash.keys()"<<mtsHash.keys();
// 	qDebug()<<"end of recursiveConstructor mts.keys()"<<mts.keys();
}


void InnerModelViewer::update()
{	
		foreach(QString key, mts.keys())
		{
			InnerModelNode *node = innerModel->getNode(key);
			if (node->parent)
			{
				mts[key]->setMatrix(QMatToOSGMat4(*node));
			}
		}
		foreach(QString key, planeMts.keys())
		{
			osg::MatrixTransform *mt = planeMts[key];
			InnerModelPlane *plane = (InnerModelPlane *)innerModel->getNode(key);
			setOSGMatrixTransformForPlane(mt, plane);
			IMVPlane *imvplane = planesHash[key];
			if (imvplane)
			{
				if (imvplane->texture and imvplane->image and imvplane->data and imvplane->dirty)
				{
					imvplane->performUpdate();
				}
			}
		}
		foreach(QString key, meshHash.keys()) 
		{
			osg::MatrixTransform *mt = meshHash[key].meshMts;// meshMts[key];
			InnerModelMesh *mesh = (InnerModelMesh *)innerModel->getNode(key);
			RTMat rtmat = RTMat();
			rtmat.setR (mesh->rx, mesh->ry, mesh->rz);
			rtmat.setTr(mesh->tx, mesh->ty, mesh->tz);
			((osg::MatrixTransform *)meshHash[key].osgmeshPaths->getChild(0))->setMatrix(osg::Matrix::scale(mesh->scalex,mesh->scaley,mesh->scalez));
			mt->setMatrix(QMatToOSGMat4(rtmat));
			
			osg::Node *osgMesh = meshHash[mesh->id].osgmeshes;
			if (!osgMesh)
				printf("Could not find %s osg.\n", mesh->meshPath.toStdString().c_str());
			osg::PolygonMode* polygonMode = osgmeshmodes[mesh->id];
			if (mesh->render == InnerModelMesh::WireframeRendering) // wireframe
				polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
			else
				polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);

			osgMesh->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		}
}


void InnerModelViewer::setOSGMatrixTransformForPlane(osg::MatrixTransform *mt, InnerModelPlane *plane)
{
	osg::Matrix r;
	r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(plane->normal(0), plane->normal(1), -plane->normal(2)));
	osg::Matrix t;
	t.makeTranslate(osg::Vec3(plane->point(0), plane->point(1), -plane->point(2)));
	mt->setMatrix(r*t);
}

void InnerModelViewer::setOSGMatrixTransformForDisplay(osg::MatrixTransform *mt, InnerModelDisplay *display)
{
	osg::Matrix r;
	r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(display->normal(0), display->normal(1), -display->normal(2)));
	osg::Matrix t;
	t.makeTranslate(osg::Vec3(display->point(0), display->point(1), -display->point(2)));
	mt->setMatrix(r*t);
}

void InnerModelViewer::reloadMesh(QString id)
{
	// Create mesh
	InnerModelMesh *mesh = (InnerModelMesh *)innerModel->getNode(id);
	if (not mesh)
	{
		printf("Internal error\n");
		return;
	}
	osg::ref_ptr<osg::Node> osgMesh = osgDB::readNodeFile(mesh->meshPath.toStdString());
	if (not osgMesh)
	{
		printf("Could not find %s osg.\n", mesh->meshPath.toStdString().c_str());
		return;
	}	
	meshHash[id].osgmeshPaths->removeChild(0, 1);
	meshHash[id].osgmeshPaths->addChild(osgMesh);
	((osg::MatrixTransform *)meshHash[id].osgmeshPaths->getChild(0))->setMatrix(osg::Matrix::scale(mesh->scalex,mesh->scaley,mesh->scalez));
	meshHash[id].osgmeshes=osgMesh;
}

osg::Geode* InnerModelViewer::getGeode(QString id)
{
	osg::Group *osgMesh = dynamic_cast<osg::Group *>(meshHash[id].osgmeshes.get());
	osg::Geode *geode=NULL;
	while (geode==NULL)
	{
		osgMesh = dynamic_cast<osg::Group *>(osgMesh->getChild(0));
		geode = dynamic_cast<osg::Geode *>(osgMesh->getChild(0));
	}
	return geode;
}


void InnerModelViewer::setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov) const
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

void InnerModelViewer::setCameraCenter(OsgView *view, const QVec eye_)
{
        eye = eye_;
        lookAt(view, eye, to, up);
}

void InnerModelViewer::setLookTowards(OsgView *view, const QVec to_, const QVec up_)
{
        to = to_;
        up = up_;
        lookAt(view, eye, to, up);
}

void InnerModelViewer::lookAt(OsgView *view, const QVec eye_, const QVec to_, const QVec up_)
{
        eye = eye_;
        to = to_;
        up = up_;
        osgGA::CameraManipulator *tb = view->getCameraManipulator();
        osg::Vec3d oeye = OsgView::qvecToVec3(eye);
        osg::Vec3d oto  = OsgView::qvecToVec3(to);
        osg::Vec3d oup  = OsgView::qvecToVec3(up);
        tb->setHomePosition(oeye, oto, oup, true);
        tb->setByMatrix(osg::Matrixf::lookAt(oeye, oto, oup));
        view->setCameraManipulator(tb);
}

// ------------------------------------------------------------------------------------------------
// Stand-alone functions
// ------------------------------------------------------------------------------------------------

osg::Vec3 QVecToOSGVec(const QVec &vec)
{
	return osg::Vec3(vec(0), vec(1), -vec(2));
}

osg::Vec4 htmlStringToOsgVec4(QString color)
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

QString osgVec4ToHtmlString(osg::Vec4 color)
{
	QString ret("#");
	QString red   = QString::number(((int)color[0]*255), 16);
	QString green = QString::number(((int)color[1]*255), 16);
	QString blue  = QString::number(((int)color[2]*255), 16);
	return ret + red + green + blue;
}

osg::Matrix QMatToOSGMat4(const RTMat &nodeB)
{
	QVec angles = nodeB.extractAnglesR();
	QVec t = nodeB.getTr();
	RTMat node = RTMat(-angles(0), -angles(1), angles(2), QVec::vec3(t(0), t(1), -t(2)));

	return osg::Matrixd( node(0,0), node(1,0), node(2,0), node(3,0),
	                     node(0,1), node(1,1), node(2,1), node(3,1),
	                     node(0,2), node(1,2), node(2,2), node(3,2),
	                     node(0,3), node(1,3), node(2,3), node(3,3) );
}

// ------------------------------------------------------------------------------------------------
// IMVPlane
// ------------------------------------------------------------------------------------------------

IMVPlane::IMVPlane(InnerModelPlane *plane, std::string imagenEntrada, osg::Vec4 valoresMaterial, float transparencia) : osg::Geode()
{
	data = NULL;
	bool constantColor = false;
	if (imagenEntrada.size() == 7)
	{
		if (imagenEntrada[0] == '#')
		{
			constantColor = true;
		}
	}

	// Open image
	image = NULL;
	//CAUTION
	//osg::TessellationHints* hints;
	osg::ref_ptr<osg::TessellationHints> hints;
	if (imagenEntrada.size()>0 and not constantColor)
	{
		if (imagenEntrada == "custom")
			image = new osg::Image();
		else
		{
			image = osgDB::readImageFile(imagenEntrada);
			if (not image)
			{
				qDebug() << "Couldn't load texture:" << imagenEntrada.c_str();
				throw "Couldn't load texture.";
			}
		}
	}
	hints = new osg::TessellationHints;
	hints->setDetailRatio(2.0f);

	//CAUTION
	//osg::Box* myBox = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), plane->width, -plane->height, plane->depth);
	osg::ref_ptr<osg::Box> myBox = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), plane->width, -plane->height, plane->depth);
// 	osg::Box* myBox = new osg::Box(QVecToOSGVec(QVec::vec3(plane->point(0),-plane->point(1),plane->point(2))), plane->width, -plane->height, plane->depth);
	planeDrawable = new osg::ShapeDrawable(myBox, hints);
	planeDrawable->setColor(htmlStringToOsgVec4(QString::fromStdString(imagenEntrada)));

	addDrawable(planeDrawable);

	if (not constantColor)
	{
		// Texture
		texture = new osg::Texture2D;
		if (image)
		{
			texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			texture->setImage(image);
			texture->setDataVariance(Object::DYNAMIC);
			texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
			texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
			texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			texture->setTextureWidth(1);
			texture->setTextureHeight(1);
		}
		texture->setResizeNonPowerOfTwoHint(false);

		// Material
		//osg::Material *material = new osg::Material();
		osg::ref_ptr<osg::Material> material = new osg::Material();
		material->setTransparency( osg::Material::FRONT_AND_BACK, transparencia);
		material->setEmission(osg::Material::FRONT, osg::Vec4(0.8, 0.8, 0.8, 0.5));
		// Assign the material and texture to the plane
		osg::StateSet *sphereStateSet = getOrCreateStateSet();
		sphereStateSet->ref();
		sphereStateSet->setAttribute(material);
#ifdef __arm__
#else
		sphereStateSet->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
#endif
		sphereStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
	}
}

IMVPlane::IMVPlane(InnerModelDisplay *plane, std::string imagenEntrada, osg::Vec4 valoresMaterial, float transparencia) : osg::Geode()
{
	data = NULL;
	bool constantColor = false;
	if (imagenEntrada.size() == 7)
	{
		if (imagenEntrada[0] == '#')
		{
			constantColor = true;
		}
	}

	// Open image
	image = NULL;
	//CAUTION
	//osg::TessellationHints* hints;
	osg::ref_ptr<osg::TessellationHints> hints;
	if (imagenEntrada.size()>0 and not constantColor)
	{
		if (imagenEntrada == "custom")
			image = new osg::Image();
		else
		{
			image = osgDB::readImageFile(imagenEntrada);
			if (not image)
			{
				qDebug() << "Couldn't load texture:" << imagenEntrada.c_str();
				throw "Couldn't load texture.";
			}
		}
	}
	hints = new osg::TessellationHints;
	hints->setDetailRatio(2.0f);

	//CAUTION
	//osg::Box* myBox = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), plane->width, -plane->height, plane->depth);
	osg::ref_ptr<osg::Box> myBox = new osg::Box(QVecToOSGVec(QVec::vec3(0,0,0)), plane->width, -plane->height, plane->depth);
// 	osg::Box* myBox = new osg::Box(QVecToOSGVec(QVec::vec3(plane->point(0),-plane->point(1),plane->point(2))), plane->width, -plane->height, plane->depth);
	planeDrawable = new osg::ShapeDrawable(myBox, hints);
	planeDrawable->setColor(htmlStringToOsgVec4(QString::fromStdString(imagenEntrada)));

	addDrawable(planeDrawable);

	if (not constantColor)
	{
		// Texture
		texture = new osg::Texture2D;
		if (image)
		{
			texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			texture->setImage(image);
			texture->setDataVariance(Object::DYNAMIC);
			texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
			texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
			texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			texture->setTextureWidth(1);
			texture->setTextureHeight(1);
		}
		texture->setResizeNonPowerOfTwoHint(false);

		// Material
		//osg::Material *material = new osg::Material();
		osg::ref_ptr<osg::Material> material = new osg::Material();
		material->setTransparency( osg::Material::FRONT_AND_BACK, transparencia);
		material->setEmission(osg::Material::FRONT, osg::Vec4(0.8, 0.8, 0.8, 0.5));
		// Assign the material and texture to the plane
		osg::StateSet *sphereStateSet = getOrCreateStateSet();
		sphereStateSet->ref();
		sphereStateSet->setAttribute(material);
#ifdef __arm__
#else
		sphereStateSet->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
#endif
		sphereStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
	}
}

IMVPlane::~IMVPlane ( )
{}

void IMVPlane::setImage(osg::Image *image_)
{
			texture = new osg::Texture2D;
			image = image_;
			if (image)
			{
				texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
				texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
				texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
				texture->setImage(image);
				texture->setDataVariance(Object::DYNAMIC);
				texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
				texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
				texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
				texture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
				texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
				texture->setTextureWidth(1);
				texture->setTextureHeight(1);
			}
			texture->setResizeNonPowerOfTwoHint(false);

			// Material
			//osg::Material *material = new osg::Material();
			osg::ref_ptr<osg::Material> material = new osg::Material();
			material->setTransparency( osg::Material::FRONT_AND_BACK, 0);
			material->setEmission(osg::Material::FRONT, osg::Vec4(0.8, 0.8, 0.8, 0.5));
			// Assign the material and texture to the plane
			osg::StateSet *sphereStateSet = getOrCreateStateSet();
			sphereStateSet->ref();
			sphereStateSet->setAttribute(material);
	#ifdef __arm__
	#else
			sphereStateSet->setTextureMode(0, GL_TEXTURE_GEN_R, osg::StateAttribute::ON);
	#endif
			sphereStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
}

void IMVPlane::updateBuffer(uint8_t *data_, int32_t width_, int32_t height_)
{
	data = data_;
	width = width_;
	height = height_;
	dirty = true;
}

void IMVPlane::performUpdate()
{
	static uint8_t *backData = NULL;

	if (dirty)
	{
		if (backData != data)
		{
#ifdef __arm__
			image->setImage(width, height, 3, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, data, osg::Image::NO_DELETE, 1);
#else
			image->setImage(width, height, 3, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE, data, osg::Image::NO_DELETE, 1);
#endif

		}
		else
		{
			image->dirty();
		}
		dirty = false;
		backData = data;
	}
}

// ------------------------------------------------------------------------------------------------
// IMVPointCloud
// ------------------------------------------------------------------------------------------------

IMVPointCloud::IMVPointCloud(std::string id_) : osg::Geode()
{
	/// ID
	id = id_;
	/// Default point size
	pointSize = 1.;
	/// Vertices
	points = new osg::Vec3Array;
	points->resize(64);
	cloudVertices = new osg::Vec3Array;
	*cloudVertices = *points;
	/// Colors
	colors = new osg::Vec4Array;
	colors->resize(64);
	colorsArray = new osg::Vec4Array;
	*colorsArray = *colors;

	/// DrawArrays
	arrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloudVertices->size());
	/// Geometry
	cloudGeometry = new osg::Geometry();
	cloudGeometry->setVertexArray(cloudVertices);
	cloudGeometry->addPrimitiveSet(arrays);
	cloudGeometry->setColorArray(colorsArray);
	cloudGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	/// Geode

	addDrawable(cloudGeometry);
}

void IMVPointCloud::update()
{
	if (points->size() != colors->size()) throw "points->size() != colors->size()";
	/// Geode 1
	removeDrawable(cloudGeometry);

	/// Arrays
	cloudVertices = new osg::Vec3Array;
	*cloudVertices = *points;
	colorsArray = new osg::Vec4Array;
	*colorsArray = *colors;

	/// DrawArrays
	arrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloudVertices->size());
	/// Geometry
	cloudGeometry = new osg::Geometry();
	cloudGeometry->setVertexArray(cloudVertices);
	cloudGeometry->addPrimitiveSet(arrays);
	cloudGeometry->getOrCreateStateSet()->setAttribute( new osg::Point(pointSize), osg::StateAttribute::ON );
	cloudGeometry->setColorArray(colorsArray);
	cloudGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	/// Geode 2
	addDrawable(cloudGeometry);
}

float IMVPointCloud::getPointSize()
{
	return pointSize;
}

void IMVPointCloud::setPointSize(float p)
{
	pointSize = p;
}

