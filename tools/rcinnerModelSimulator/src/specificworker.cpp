/*
 *    Caopyright (C) 2006-2014 by RoboLab - University of Extremadura
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

// Simulator includes
#include "specificworker.h"


// #define INNERMODELMANAGERDEBUG

/**
 *\brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& _mprx, Ice::CommunicatorPtr _communicator, const char *_innerModelXML, int ms) : GenericWorker(_mprx)
{
	communicator = _communicator;
	laserDataCartArray.clear();

	// Initialize InnerModel from file
	innerModel = std::make_shared<InnerModel>(_innerModelXML);
	
	qDebug() << __FILE__ << __FUNCTION__ << "InnerModel read";
	
	//add name of .xml to window
	setWindowTitle(windowTitle() + "\t" + _innerModelXML);

	// Initialize the Inner Model Viewer
	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	viewer = new OsgView(frameOSG);
	
	// create InnerModelViewer
	imv = std::make_shared<InnerModelViewer>(innerModel, "root", viewer->getRootGroup());

	// view manipulator
	manipulator = new osgGA::TrackballManipulator;
	// 	manipulator->setHomePosition(osg::Vec3d(0, 10000, 0), osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, -10000), true);
	viewer->setCameraManipulator(manipulator, true);
	
	// Add mouse pick handler to publish 3D coordinates
	if (rcis_mousepicker_proxy)
		viewer->addEventHandler(new PickHandler(rcis_mousepicker_proxy));

	// Restore previous camera position
	settings = new QSettings("RoboComp", "RCIS");
	QString path(_innerModelXML);
	if (path == settings->value("path").toString() )
	{
		//restore matrix view
		QStringList l = settings->value("matrix").toStringList();
		if (l.size() > 0)
		{
			osg::Matrixd m;
			for (int i=0; i<4; i++ )
				for (int j=0; j<4; j++ )
					m(i,j)=l.takeFirst().toDouble();
			manipulator->setByMatrix(m);
		}
		else
			setTopPOV();
	}
	else
		settings->setValue("path",path);
	qDebug() << __FILE__ << __FUNCTION__ << "InnerModelViewer created";

	// Connect all the SIGNALS to SLOTS
	connect(topView,   SIGNAL(clicked()), this, SLOT(setTopPOV()));
	connect(leftView,  SIGNAL(clicked()), this, SLOT(setLeftPOV()));
	connect(rightView, SIGNAL(clicked()), this, SLOT(setRightPOV()));
	connect(frontView, SIGNAL(clicked()), this, SLOT(setFrontPOV()));
	connect(backView,  SIGNAL(clicked()), this, SLOT(setBackPOV()));
	connect(sp_lightx,  SIGNAL(valueChanged(double)), this, SLOT(setLigthx(double)));
	connect(sp_lighty,  SIGNAL(valueChanged(double)), this, SLOT(setLigthy(double)));
	connect(sp_lightz,  SIGNAL(valueChanged(double)), this, SLOT(setLigthz(double)));
	connect(actionObject, SIGNAL(triggered()), this, SLOT(objectTriggered()));
	connect(actionVisual, SIGNAL(triggered()), this, SLOT(visualTriggered()));

	// Additional widgets
	objectTriggered();
	visualTriggered();
	
	//Init viewer
	viewer->realize();
	//viewer->setThreadingModel( osgViewer::ViewerBase::ThreadPerCamera);
	
	//Initialize Ice interfaces
	servers.init(innerModel, imv, worker, communicator); 

	// Initialize the timer
	setPeriod(ms);	
	
	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
}

void SpecificWorker::compute()
{
	auto elapsed = fps.print();
	
	guard gl(innerModel->mutex);
	
		updateCameras();
		updateLasers();
 		updateJoints(elapsed/1000.0f);
 		//updateTouchSensors();  //BUGGYYYYYYYYYYYY

		#ifdef INNERMODELMANAGERDEBUG
			printf("Elapsed time: %d\n", elapsed);
		#endif
		
		// Shutdown empty servers
		servers.shutdownEmptyServers();
		
		// Resize world widget if necessary, and render the world
		if (viewer->size() != frameOSG->size())
			viewer->setFixedSize(frameOSG->width(), frameOSG->height());
		imv->update();
		
		//osg render
		viewer->frame();
}

		
//////////////////////////////////////////////////////////////////////
/// Updates
////////////////////////////////////////////////////////////////////////

void SpecificWorker::updateCameras()
{
	auto i = imv->cameras.constBegin();
	{
		while (i != imv->cameras.constEnd())
		{
			RTMat rt= innerModel->getTransformationMatrix("root",i.key());
			// Put camera in its position
			imv->cameras[i.key()].viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));

			for (int n=0; n<imv->cameras.size() ; ++n)
				imv->cameras[i.key()].viewerCamera->frame();
			i++;
		}
	}	
}

void SpecificWorker::updateLasers()
{
	guard gl(innerModel->mutex);
	
	// Delete existing lasers
	for (auto laser = imv->lasers.begin(); laser != imv->lasers.end(); laser++)
	{
		if (laser->osgNode->getNumChildren() > 0)
			laser->osgNode->removeChild(0, laser->osgNode->getNumChildren());
	}
	
	// Laser
	for (auto laser = imv->lasers.begin(); laser != imv->lasers.end(); laser++)
	{
		std::string id = laser->laserNode->id.toStdString();
		
		if( laserDataCartArray.count(id) == 0)
		{
			osg::Vec3Array *v = new osg::Vec3Array();
			v->resize(laser->laserNode->measures+1);
			laserDataCartArray.insert(std::make_pair(id,v));
		}

		// create and insert laser data
		laserDataArray.insert(std::make_pair(laser->laserNode->id.toStdString(), LASER_createLaserData(laser.value())));

		// create and insert laser shape
		if (false) // DRAW LASER
		{
			osg::ref_ptr<osg::Node> p = nullptr;
			if(id == "laserSecurity")
			{
				p = viewer->addPolygon(*(laserDataCartArray[id]), osg::Vec4(0.,0.,1.,0.4));
			}
			else
			{
				p = viewer->addPolygon(*(laserDataCartArray[id]));
			}
			if (p != nullptr)
			{
				laser->osgNode->addChild(p);
			}
		}
	}
}

// Update all the joint positions
void SpecificWorker::updateJoints(const float delta)
{
	// 	printf("%s: %d\n", __FILE__, __LINE__);
	QHash<QString, JointMovement>::const_iterator iter;
	for (iter = jointMovements.constBegin() ; iter != jointMovements.constEnd() ; ++iter)
	{
		InnerModelNode *node = innerModel->getNode(iter.key());
		InnerModelJoint *ajoint;
		InnerModelPrismaticJoint *pjoint;
		if ((ajoint = dynamic_cast<InnerModelJoint*>(node)) != NULL)
		{
			const float angle = ajoint->getAngle();
			const float amount = fminf(fabsf(iter->endPos - angle), iter->endSpeed  *delta);
			switch (iter->mode)
			{
			case JointMovement::FixedPosition:
				ajoint->setAngle(iter->endPos);
				break;
			case JointMovement::TargetPosition:
				if (iter->endPos > angle)
					ajoint->setAngle(angle + amount);
				else if (iter->endPos < angle)
					ajoint->setAngle(angle - amount);
				break;
			case JointMovement::TargetSpeed:
				ajoint->setAngle(angle + iter->endSpeed  *delta);
				break;
			default:
				break;
			}
		}
		else if ((pjoint = dynamic_cast<InnerModelPrismaticJoint*>(node)) != NULL)
		{
			pjoint->setPosition(iter->endPos);
		}
	}
}

void SpecificWorker::updateTouchSensors()
{
	//std::map<uint32_t, TouchSensorServer>::iterator touchIt;
	//for (touchIt = servers.touch_servers.begin(); touchIt != servers.touch_servers.end(); touchIt++)
	for(auto &[k, v] : servers.hMaps.getMap<TouchSensorServer>())	
		for(auto s : v.sensors)
		//for (uint32_t sss=0; sss<touchIt->second.sensors.size(); sss++)
		{
			// 	TouchSensorI *interface;
			// touchIt->interface->sensorMap[touchIt->sensors[sss].id].value = XXX
			//InnerModelTouchSensor *sensorr = touchIt->second.sensors[sss];
			auto idd = s->id.toStdString();
			//std::string idd = sensorr->id.toStdString();
		}
}

// ------------------------------------------------------------------------------------------------
// NO DEBERIAN USARSE PORQUE VIOLAN LA LOGICA DEL MUTEX
// ------------------------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////////////////
///// Aux
////////////////////////////////////////////////////////////////////////////////////////
osg::Group *SpecificWorker::getRootGroup()
{
	guard gl(innerModel->mutex);
	return viewer->getRootGroup();
}
std::shared_ptr<InnerModel> SpecificWorker::getInnerModel()
{
	guard gl(innerModel->mutex);
	return innerModel;
}

std::shared_ptr<InnerModelViewer> SpecificWorker::getInnerModelViewer()
{
	guard gl(innerModel->mutex);
	return imv;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Refills laserData with new values
RoboCompLaser::TLaserData SpecificWorker::LASER_createLaserData(const IMVLaser &laser)
{
	guard gl(innerModel->mutex);
  // 		printf("osg threads running... %d\n", viewer->areThreadsRunning());
	static RoboCompLaser::TLaserData laserData;
	int measures = laser.laserNode->measures;
	std::string id = laser.laserNode->id.toStdString();
	float iniAngle = -laser.laserNode->angle/2;
	float finAngle = laser.laserNode->angle/2;
	float_t maxRange = laser.laserNode->max;
	laserData.resize(measures);

	double angle = finAngle;  //variable to iterate angle increments
	
	//El punto inicial es el origen del láser
	auto laserNode = innerModel->getNode<InnerModelLaser>(id);
	const osg::Vec3 P = QVecToOSGVec(laserNode->laserTo(std::string("root"), 0, 0));
	
	const float incAngle = (fabs(iniAngle)+fabs(finAngle)) / (float)measures;
	osg::Vec3 Q,R;

	for (int i=0 ; i<measures; i++)
	{
		laserData[i].angle = angle;
		laserData[i].dist = maxRange;
		
		//laserDataCartArray[id]->operator[](i) = QVecToOSGVec(QVec::vec3(maxRange*sin(angle), 0, maxRange*cos(angle)));
		
		//Calculamos el punto destino
		Q = QVecToOSGVec(laserNode->laserTo(std::string("root"), maxRange, angle));
		//Creamos el segmento de interseccion
		osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, P, Q);
		osgUtil::IntersectionVisitor visitor(intersector.get());

		/// Pasando el visitor al root
		viewer->getRootGroup()->accept(visitor);

		if (intersector->containsIntersections() and id!="laserSecurity")
		{
			osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
			R = result.getWorldIntersectPoint(); // in world space

			R.x() = R.x() - P.x();
			R.y() = R.y() - P.y();
			R.z() = R.z() - P.z();
			const float dist = sqrt(R.x() *R.x() + R.y() *R.y() + R.z() *R.z());

			if (dist <= maxRange)
			{
				laserData[i].dist = dist;//*1000.;
				laserDataCartArray[id]->operator[](i) = QVecToOSGVec(laserNode->laserTo(id, dist, laserData[i].angle));
			}
		}
		else
		{
			laserDataCartArray[id]->operator[](i) = QVecToOSGVec(laserNode->laserTo(id, maxRange, laserData[i].angle));
		}
		angle -= incAngle;
	}
	return laserData;
}

/*

// Refills touch sensor with new values
RoboCompTouchSensor::SensorMap TOUCH_createTouchData(const IMVLaser &laser)
{
}

*/
////////////////////////////////////////////////////////////////////////////////////////////////
///// UTILITIES
//////////////////////////////////////////////////////////////////////////////////////////////////

InnerModelNode* SpecificWorker::getNode(const QString &id, const QString &msg)
{
	guard gl(innerModel->mutex);
	InnerModelNode *node = innerModel->getNode(id);
	if (node==NULL)
	{
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NonExistingNode;
		std::ostringstream oss;
		oss << msg.toStdString() << " error: Node " << id.toStdString() << " does not exist.";
		err.text = oss.str();
		throw err;
	}
	else
	{
		return node;
	}
}

void SpecificWorker::checkOperationInvalidNode(InnerModelNode *node,QString msg)
{
	if (node==NULL)
	{
		#ifdef INNERMODELMANAGERDEBUG
					qDebug() <<msg<<node->id<<"is not transform type";
		#endif
			RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << node->id.toStdString() <<" is not of the type require";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::checkNodeAlreadyExists(const QString &id, const QString &msg)
{
	if (innerModel->getIDKeys().contains(id))
	{
		#ifdef INNERMODELMANAGERDEBUG
			qDebug("item already exist. %s\n", id.toStdString().c_str());
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NodeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << id.toStdString() << " already exists.";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::checkInvalidMeshValues(RoboCompInnerModelManager::meshType m, QString msg)
{
	///check Scale
	osg::Node *osgMesh = osgDB::readNodeFile(m.meshPath);
	if (m.scaleX<0.0 or m.scaleY <0.0 or m.scaleZ <0.0)
	{
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<"--- Fatal:"<<msg<<"Scale can not be negative";
		qDebug() <<"m.scaleX "<<m.scaleX<<"m.scaleY"<<m.scaleY<<"m.scaleZ"<<m.scaleZ;
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidValues;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Scale (" << m.scaleX << ", " << m.scaleY << ", " << m.scaleZ << ") is invalid.";
		err.text = oss.str();
		throw err;
	}
	///check valid osg Node.
	else if (osgMesh==NULL)
	{
		#ifdef INNERMODELMANAGERDEBUG
				qDebug() <<"--- Fatal:"<<msg<<"meshPath:"<<QString::fromStdString(m.meshPath) <<"does not exist or no it is a type valid for his OpenSceneGraph.";
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidPath;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: meshPath: " << m.meshPath << ", " <<"does not exist or no it is a type valid for his OpenSceneGraph.";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::AttributeAlreadyExists(InnerModelNode *node, QString attributeName, QString msg)
{
	if (node->attributes.contains(attributeName))
	{
		#ifdef INNERMODELMANAGERDEBUG
				qDebug("attribute already exist. %s\n", attributeName.toStdString().c_str());
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " already exists." <<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::NonExistingAttribute(InnerModelNode *node, QString attributeName, QString msg)
{
	if (node->attributes.contains(attributeName) ==false)
	{
#ifdef INNERMODELMANAGERDEBUG
		qDebug("attribute NO exist. %s\n", attributeName.toStdString().c_str());
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " NO exists."<<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::getRecursiveNodeInformation(RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, InnerModelNode *node)
{
	/// Add current node information
	RoboCompInnerModelManager::NodeInformation ni;
	ni.id = node->id.toStdString();


	if (node->parent)
	{
		ni.parentId = node->parent->id.toStdString();
	}
	else
	{
		ni.parentId = "";
	}
	ni.nType = getNodeType(node);

	RoboCompInnerModelManager::AttributeType a;
	foreach (const QString &str, node->attributes.keys())
	{
		a.type=node->attributes.value(str).type.toStdString();
		a.value=node->attributes.value(str).value.toStdString();
		ni.attributes[str.toStdString()]=a;
	}
	nodesInfo.push_back(ni);

	/// Recursive call for all children
	QList<InnerModelNode *>::iterator child;
	for (child = node->children.begin(); child != node->children.end(); child++)
	{
		getRecursiveNodeInformation(nodesInfo, *child);
	}
}

RoboCompInnerModelManager::NodeType SpecificWorker::getNodeType(InnerModelNode *node)
{
	if (dynamic_cast<InnerModelJoint*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Joint;
	}
	else if (dynamic_cast<InnerModelTouchSensor*>(node) != NULL)
	{
		return RoboCompInnerModelManager::TouchSensor;
	}
	else if (dynamic_cast<InnerModelDifferentialRobot*>(node) != NULL)
	{
		return RoboCompInnerModelManager::DifferentialRobot;
	}
	else if (dynamic_cast<InnerModelOmniRobot*>(node) != NULL)
	{
		return RoboCompInnerModelManager::OmniRobot;
	}
	else if (dynamic_cast<InnerModelPlane*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Plane;
	}
	else if (dynamic_cast<InnerModelDisplay*>(node) != NULL)
	{
		return RoboCompInnerModelManager::DisplayII;
	}
	else if (dynamic_cast<InnerModelRGBD*>(node) != NULL)
	{
		return RoboCompInnerModelManager::RGBD;
	}
	else if (dynamic_cast<InnerModelCamera*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Camera;
	}
	else if (dynamic_cast<InnerModelIMU*>(node) != NULL)
	{
		return RoboCompInnerModelManager::IMU;
	}
	else if (dynamic_cast<InnerModelLaser*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Laser;
	}
	else if (dynamic_cast<InnerModelMesh*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Mesh;
	}
	else if (dynamic_cast<InnerModelPointCloud*>(node) != NULL)
	{
		return RoboCompInnerModelManager::PointCloud;
	}
	else if (dynamic_cast<InnerModelTransform*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Transform;
	}
	else
	{
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InternalError;
		std::ostringstream oss;
		oss << "RoboCompInnerModelManager::getNodeType() error: Type of node " << node->id.toStdString() << " is unknown.";
		err.text = oss.str();
		throw err;
	}
}

// Cambia el color de un mesh
void SpecificWorker::cambiaColor(QString id, osg::Vec4 color)
{
	osg::Node *node = imv->meshHash[id].osgmeshes;//imv->osgmeshes[id];
	node = dynamic_cast<osg::Group*>(imv->meshHash[id].osgmeshes.get())->getChild(0);
	if (node)
	{
		osg::Material *mat = new osg::Material;
		mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
		node->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::OVERRIDE);
	}
}

// Devuelve el colorido inicial a un mesh
void SpecificWorker::devuelveColor(QString id)
{
	osg::Node *node = imv->meshHash[id].osgmeshes;
	node = dynamic_cast<osg::Group*>(imv->meshHash[id].osgmeshes.get())->getChild(0);
	if (node)
	{
		osg::Material *mat = new osg::Material;
		node->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}
}

// Activa / desactiva las luces
void SpecificWorker::changeLigthState(bool apagar)
{
	//QMutexLocker vm(worker->viewerMutex);
	guard gl(innerModel->mutex);
	osg::StateSet *state = viewer->getRootGroup()->getOrCreateStateSet();

	if(apagar)
	{/// apagar luces
		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	}
	else
	{/// encender luces
		state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	}
}


void SpecificWorker::objectTriggered()
{
	if (actionObject->isChecked())
	{
		OBJECTWidget->show();
	}
	else
	{
		OBJECTWidget->hide();
	}
}

void SpecificWorker::visualTriggered()
{
	if (actionVisual->isChecked())
	{
		VISUALWidget->show();
	}
	else
	{
		VISUALWidget->hide();
	}
}


// ------------------------------------------------------------------------------------------------
// Slots
// ------------------------------------------------------------------------------------------------


void SpecificWorker::setTopPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
}


void SpecificWorker::setFrontPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::FRONT_POV);
}


void SpecificWorker::setBackPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::BACK_POV);
}


void SpecificWorker::setLeftPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::LEFT_POV);
}


void SpecificWorker::setRightPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::RIGHT_POV);
}


void SpecificWorker::closeEvent(QCloseEvent *event)
{
	event->accept();
	osg::Matrixd m = manipulator->getMatrix();
	QString s="";
	QStringList l;
	for (int i=0; i<4; i++ )
	{
		for (int j=0; j<4; j++ )
		{
			l.append(s.number(m(i,j)));
		}
	}
	settings->setValue("matrix", l);
	settings->sync();

	exit(EXIT_SUCCESS);
}

void SpecificWorker::setLigthx(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(v,p.y(),p.z(),p.w());
	viewer->getLight()->setPosition(p);
}

void SpecificWorker::setLigthy(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(p.x(),v,p.z(),p.w());
	viewer->getLight()->setPosition(p);
}

void SpecificWorker::setLigthz(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(p.x(),p.y(),v,p.w());
	viewer->getLight()->setPosition(p);
}




// ------------------------------------------------------------------------------------------------
// ICE interfaces
// ------------------------------------------------------------------------------------------------

// void SpecificWorker::checkPoseCollision(QString node,QString msg)
//
//{
// 	///por cada mesh descendiente chequear colisiones con todo el mundo menos con sus mesh hermanas
// #ifdef INNERMODELMANAGERDEBUG
// 	qDebug() <<"checkPoseCollision"<<msg<<node<<"A?";
// #endif
// 	QStringList l;
// 	l.clear();
//
// 	innerModel->getSubTree(innerModel->getNode(node),&l);
//
// 	/// Checking
// 	foreach (QString n, l)
// 	{
// 		/// Replicate plane removals
// 		if (imv->meshHash.contains(n))
// 		{
// 			QList <QString> excludingList;
// 			excludingList.clear();
// 			detectarColision1toN(n,excludingList,msg);
//
// 		}
//
// // 		/// Replicate plane removals
// // 		if (imv->planeMts.contains(n))
// // 		{
// // 			while (imv->planeMts[n]->getNumParents() > 0)
// // 			{
// // 				((osg::Group *)(imv->planeMts[n]->getParent(0)))->removeChild(imv->planeMts[n]);
// // 			}
// // 			imv->planeMts.remove(n);
// // 			imv->planesHash.remove(n);
// // 		}
// 	}
// }
