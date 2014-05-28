/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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

#ifdef USE_BULLET
#include "physicsMotion.h"
#include "physicsWorld.h"
#endif

// Qt includes
#include <QDropEvent>
#include <QEvent>
#include <QGLWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QTime>
#include <QWidget>

// OSG includes
#include <osg/io_utils>
#include <osg/BoundingBox>
#include <osg/LineWidth>
#include <osg/Matrixd>
#include <osg/PolygonMode>
#include <osg/TriangleFunctor>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osgText/Font>
#include <osgText/Text>
#include <osgUtil/IntersectVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

// #define INNERMODELMANAGERDEBUG


struct JointMovement
{
	float endPos;
	float endSpeed;
	float maxAcc;
	enum { FixedPosition, TargetPosition, TargetSpeed } mode;
};



struct SpecificWorker::Data
{
	// World
	SpecificWorker* worker;
	InnerModel* innerModel;
	InnerModelViewer* imv;
	OsgView* viewer;
	osgGA::TrackballManipulator* manipulator;
	


	#ifdef USE_BULLET
	World* world;
	#endif

	// Handlers
	Ice::CommunicatorPtr communicator;
	std::map<uint32_t, JointMotorServer> jm_servers;
	std::map<uint32_t, TouchSensorServer> touch_servers;
	std::map<uint32_t, LaserServer> laser_servers;
	std::map<uint32_t, RGBDServer> rgbd_servers;
	std::map<uint32_t, IMUServer> imu_servers;
	std::map<uint32_t, DifferentialRobotServer> dfr_servers;

	QList <JointMotorServer *> jointServersToShutDown;

	// Camera
	
	// DifferentialRobot
	
	// InnerModelManager
	
	// IMU
	DataImu data_imu;
	
	// JointMotor
	QHash<QString, JointMovement> jointMovements;
	
	// Laser
	QMap<QString, RoboCompLaser::TLaserData> laserDataArray;
	QMap<QString, osg::Vec3Array*> laserDataCartArray;
	
	// RGBD
	
	// ------------------------------------------------------------------------------------------------
	// Private methods
	// ------------------------------------------------------------------------------------------------
	
	// Refills laserData with new values
	RoboCompLaser::TLaserData LASER_createLaserData (const IMVLaser &laser)
	{
		// 	QMutexLocker locker (mutex);
		static RoboCompLaser::TLaserData laserData;
		int measures = laser.laserNode->measures;
		QString id = laser.laserNode->id;
		float iniAngle = -laser.laserNode->angle/2;
		float finAngle = laser.laserNode->angle/2;
		float_t maxRange = laser.laserNode->max;

		laserData.resize (measures);

		double angle = finAngle;  //variable to iterate angle increments
		//El punto inicial es el origen del láser
		const osg::Vec3 P = QVecToOSGVec (innerModel->laserToWorld (id, 0, 0));
		const float incAngle = (fabs (iniAngle) + fabs (finAngle)) / (float) measures;
		osg::Vec3 Q,R;

		for (int i=0 ; i<measures; i++)
		{
			laserData[i].angle = angle;
			laserData[i].dist = maxRange;//*1000.;
			laserDataCartArray[id]->operator[] (i) = QVecToOSGVec (innerModel->laserTo (id, id, angle, maxRange));

			//Calculamos el punto destino
			Q = QVecToOSGVec (innerModel->laserTo ("root", id, maxRange, angle));
			//Creamos el segmento de interseccion
			osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector (osgUtil::Intersector::MODEL, P, Q);
			osgUtil::IntersectionVisitor visitor (intersector.get());

			/// Pasando el visitor al root
			viewer->getRootGroup()->accept (visitor);

			if (intersector->containsIntersections() and id!="laserSecurity")
			{
				osgUtil::LineSegmentIntersector::Intersection result = * (intersector->getIntersections().begin());
				R = result.getWorldIntersectPoint(); // in world space

				R.x() = R.x() - P.x();
				R.y() = R.y() - P.y();
				R.z() = R.z() - P.z();
				const float dist = sqrt (R.x() *R.x() + R.y() *R.y() + R.z() *R.z());

				if (dist <= maxRange)
				{
					laserData[i].dist = dist;//*1000.;
					laserDataCartArray[id]->operator[] (i) = QVecToOSGVec (innerModel->laserTo (id, id, dist, laserData[i].angle));
				}
			}
			else
			{
				laserDataCartArray[id]->operator[] (i) = QVecToOSGVec (innerModel->laserTo (id, id, maxRange, laserData[i].angle));
			}
			angle -= incAngle;
		}

		///what does it mean? the point of the laser robot.
		laserDataCartArray[id]->operator[] (measures) = QVecToOSGVec (innerModel->laserTo (id, id, 0.0001, 0.001));

		return laserData;
	}

/*




	// Refills touch sensor with new values
	RoboCompTouchSensor::SensorMap TOUCH_createTouchData (const IMVLaser &laser)
	{
	}

*/




	///--- useful functions.
	InnerModelNode *getNode (const QString &id, const QString &msg)
	{
		InnerModelNode *node = innerModel->getNode (id);
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


	void checkOperationInvalidNode (InnerModelNode *node,QString msg)
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


	void checkNodeAlreadyExists (const QString &id, const QString &msg)
	{
		if (innerModel->getIDKeys().contains (id))
		{
			#ifdef INNERMODELMANAGERDEBUG
				qDebug ("item already exist. %s\n", id.toStdString().c_str());
			#endif
			RoboCompInnerModelManager::InnerModelManagerError err;
			err.err = RoboCompInnerModelManager::NodeAlreadyExists;
			std::ostringstream oss;
			oss <<msg.toStdString() <<" error: Node " << id.toStdString() << " already exists.";
			err.text = oss.str();
			throw err;
		}
	}


	void checkInvalidMeshValues (RoboCompInnerModelManager::meshType m, QString msg)
	{
		///check Scale
		osg::Node *osgMesh = osgDB::readNodeFile (m.meshPath);
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
			qDebug() <<"--- Fatal:"<<msg<<"meshPath:"<<QString::fromStdString (m.meshPath) <<"does not exist or no it is a type valid for his OpenSceneGraph.";
	#endif

			RoboCompInnerModelManager::InnerModelManagerError err;
			err.err = RoboCompInnerModelManager::InvalidPath;
			std::ostringstream oss;
			oss <<msg.toStdString() <<" error: meshPath: " << m.meshPath << ", " <<"does not exist or no it is a type valid for his OpenSceneGraph.";
			err.text = oss.str();
			throw err;
		}
	}


	void AttributeAlreadyExists (InnerModelNode* node, QString attributeName, QString msg)
	{
		if (node->attributes.contains (attributeName))
		{
	#ifdef INNERMODELMANAGERDEBUG
			qDebug ("attribute already exist. %s\n", attributeName.toStdString().c_str());
	#endif
			RoboCompInnerModelManager::InnerModelManagerError err;
			err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
			std::ostringstream oss;
			oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " already exists." <<" in node "<<node->id.toStdString();
			err.text = oss.str();
			throw err;
		}
	}


	void NonExistingAttribute (InnerModelNode* node, QString attributeName, QString msg)
	{
		if (node->attributes.contains (attributeName) ==false)
		{
	#ifdef INNERMODELMANAGERDEBUG
			qDebug ("attribute NO exist. %s\n", attributeName.toStdString().c_str());
	#endif
			RoboCompInnerModelManager::InnerModelManagerError err;
			err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
			std::ostringstream oss;
			oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " NO exists."<<" in node "<<node->id.toStdString();
			err.text = oss.str();
			throw err;
		}
	}


	void getRecursiveNodeInformation (RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, InnerModelNode* node)
	{
		/// Add current node information
		RoboCompInnerModelManager::NodeInformation ni;
		ni.id = node->id.toStdString();

		if (node->parent)
		{
			ni.parentId = node->parent->id.toStdString();
		}else
		{
			ni.parentId = "";
		}
		ni.nType = getNodeType (node);

		RoboCompInnerModelManager::AttributeType a;
		foreach (const QString &str, node->attributes.keys())
		{
			a.type=node->attributes.value (str).type.toStdString();
			a.value=node->attributes.value (str).value.toStdString();
			ni.attributes[str.toStdString()]=a;
		}
		nodesInfo.push_back (ni);

		/// Recursive call for all children
		QList<InnerModelNode *>::iterator child;
		for (child = node->children.begin(); child != node->children.end(); child++)
		{
			getRecursiveNodeInformation (nodesInfo, *child);
		}
	}


	RoboCompInnerModelManager::NodeType getNodeType (InnerModelNode* node)
	{
		if (dynamic_cast<InnerModelJoint*> (node) != NULL)
		{
			return RoboCompInnerModelManager::Joint;
		}
		else if (dynamic_cast<InnerModelTouchSensor*> (node) != NULL)
		{
			return RoboCompInnerModelManager::TouchSensor;
		}
		else if (dynamic_cast<InnerModelDifferentialRobot*> (node) != NULL)
		{
			return RoboCompInnerModelManager::DifferentialRobot;
		}
		else if (dynamic_cast<InnerModelPlane*> (node) != NULL)
		{
			return RoboCompInnerModelManager::Plane;
		}
		else if (dynamic_cast<InnerModelRGBD*> (node) != NULL)
		{
			return RoboCompInnerModelManager::RGBD;
		}
		else if (dynamic_cast<InnerModelCamera*> (node) != NULL)
		{
			return RoboCompInnerModelManager::Camera;
		}
		else if (dynamic_cast<InnerModelIMU*> (node) != NULL)
		{
			return RoboCompInnerModelManager::IMU;
		}
		else if (dynamic_cast<InnerModelLaser*> (node) != NULL)
		{
			return RoboCompInnerModelManager::Laser;
		}
		else if (dynamic_cast<InnerModelMesh*> (node) != NULL)
		{
			return RoboCompInnerModelManager::Mesh;
		}
		else if (dynamic_cast<InnerModelPointCloud*> (node) != NULL)
		{
			return RoboCompInnerModelManager::PointCloud;
		}
		else if (dynamic_cast<InnerModelTransform*> (node) != NULL)
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
	void cambiaColor (QString id, osg::Vec4 color)
	{
		osg::Node *node = imv->meshHash[id].osgmeshes;//imv->osgmeshes[id];
		node = dynamic_cast<osg::Group*> (imv->meshHash[id].osgmeshes)->getChild (0);
		if (node)
		{
			osg::Material* mat = new osg::Material;
			mat->setDiffuse (osg::Material::FRONT_AND_BACK, color);
			node->getOrCreateStateSet()->setAttributeAndModes (mat, osg::StateAttribute::OVERRIDE);
		}
	}


	// Devuelve el colorido inicial a un mesh
	void devuelveColor (QString id)
	{
		osg::Node *node = imv->meshHash[id].osgmeshes;
		node = dynamic_cast<osg::Group*> (imv->meshHash[id].osgmeshes)->getChild (0);
		if (node)
		{
			osg::Material* mat = new osg::Material;
			node->getOrCreateStateSet()->setAttributeAndModes (mat, osg::StateAttribute::ON);
		}
	}


	// Activa / desactiva las luces
	void changeLigthState (bool apagar)
	{
		osg::StateSet* state = viewer->getRootGroup()->getOrCreateStateSet();

		switch (apagar)
		{
		case true: /// apagar luces
			state->setMode (GL_LIGHTING, osg::StateAttribute::OFF);
			break;
		case false: /// encender luces
			state->setMode (GL_LIGHTING, osg::StateAttribute::ON);
			break;
		}
	}
	
	
	// Update all the joint positions
	void updateJoints(const float delta)
	{
		QHash<QString, JointMovement>::const_iterator iter;
		for (iter = jointMovements.constBegin() ; iter != jointMovements.constEnd() ; ++iter)
		{
			InnerModelJoint* joint = this->innerModel->getJoint (iter.key());
			const float angle = joint->getAngle();
			const float amount = fminf (fabsf (iter->endPos - angle), iter->endSpeed * delta);
			switch (iter->mode)
			{
			case JointMovement::FixedPosition:
				joint->setAngle(iter->endPos);
				break;
			case JointMovement::TargetPosition:
				if (iter->endPos > angle)
					joint->setAngle(angle + amount);
				else if (iter->endPos < angle)
					joint->setAngle(angle - amount);
				break;
			case JointMovement::TargetSpeed:
				joint->setAngle (angle + iter->endSpeed * delta);
				break;
			default:
				break;
			}
		}
	}
	
	void updateTouchSensors()
	{
		std::map<uint32_t, TouchSensorServer>::iterator touchIt;
		for (touchIt=touch_servers.begin(); touchIt!= touch_servers.end(); touchIt++)
		{
			for (uint32_t sss=0; sss<touchIt->second.sensors.size(); sss++)
			{
				// 	TouchSensorI *interface;
				// touchIt->interface->sensorMap[touchIt->sensors[sss].id].value = XXX
				InnerModelTouchSensor *sensorr = touchIt->second.sensors[sss];
				std::string idd = sensorr->id.toStdString();
				printf("%d: %s (%f)\n",

					touchIt->second.port,
							 
					idd.c_str(), 
					 
					touchIt->second.interface->sensorMap[idd].value

				);
			}
		}
	}
	

	void addDFR (InnerModelDifferentialRobot* node)
	{
		const uint32_t port = node->port;
		if (dfr_servers.count(port) == 0)
		{
			dfr_servers.insert(std::pair<uint32_t, DifferentialRobotServer>(port, DifferentialRobotServer(communicator, worker, port)));
		}
		dfr_servers.at(port).add(node);
	}


	void addIMU(InnerModelIMU* node)
	{
		const uint32_t port = node->port;
		if (imu_servers.count(port) == 0)
		{
			imu_servers.insert(std::pair<uint32_t, IMUServer>(port, IMUServer(communicator, worker, port)));
		}
		imu_servers.at(port).add(node);
	}


	void addJM(InnerModelJoint* node)
	{
		const uint32_t port = node->port;
		if (jm_servers.count(port) == 0)
		{
			jm_servers.insert(std::pair<uint32_t, JointMotorServer>(port, JointMotorServer(communicator, worker, port)));
		}
		jm_servers.at(port).add(node);
	}

	void addTouch(InnerModelTouchSensor* node)
	{
		const uint32_t port = node->port;
		if (touch_servers.count(port) == 0)
		{
			touch_servers.insert(std::pair<uint32_t, TouchSensorServer>(port, TouchSensorServer(communicator, worker, port)));
		}
		touch_servers.at(port).add(node);
	}


	void addLaser (InnerModelLaser* node)
	{
		const uint32_t port = node->port;
		if (laser_servers.count(port) == 0)
		{
			laser_servers.insert(std::pair<uint32_t, LaserServer>(port, LaserServer(communicator, worker, port)));
		}
		laser_servers.at(port).add(node);
	}


	void addRGBD (InnerModelRGBD* node)
	{
		const uint32_t port = node->port;
		if (rgbd_servers.count(port) == 0)
		{
			rgbd_servers.insert(std::pair<uint32_t, RGBDServer>(port, RGBDServer(communicator, worker, port)));
		}
		rgbd_servers.at(port).add(node);
	}

	void removeJM(InnerModelJoint* node)
	{
		std::map<uint32_t, JointMotorServer>::iterator it;
		for (it = jm_servers.begin(); it != jm_servers.end(); ++it)
		{
			it->second.remove(node);
			// TODO: arreglar
	// 		if (it->second.empty())
	// 		{
	// 			worker->scheduleShutdown(&(it->second));
	// 			servers.erase(it);
	// 		}
		}
	}


	void includeLasers()
	{
		QHash<QString, IMVLaser>::const_iterator it;
		for (it = imv->lasers.constBegin() ; it != imv->lasers.constEnd() ; ++it)
		{
			qDebug() << it.key() << ": ";
			printf(" %p\n", it.value().laserNode);
			addLaser(it.value().laserNode);
		}
	}
	
	
	void includeRGBDs()
	{
		QHash<QString, IMVCamera>::const_iterator it;
		for (it = imv->cameras.constBegin() ; it != imv->cameras.constEnd() ; ++it)
		{
			addRGBD(it.value().RGBDNode);
		}
	}
	
	
	void walkTree (InnerModelNode* node = NULL)
	{
		if (node == NULL)
		{
			node = innerModel->getRoot();
			//std::cout << "ROOT: " << (void *)node << "  " << (uint64_t)node << std::endl;
		}
		else
		{
			//std::cout << "nrml: " << (void *)node << "  " << (uint64_t)node << std::endl;
		}

		QList<InnerModelNode*>::iterator it;
		for (it=node->children.begin(); it!=node->children.end(); ++it)
		{
			//std::cout << "  --> " << (void *)*it << "  " << (uint64_t)*it << std::endl;
			InnerModelDifferentialRobot* differentialNode = dynamic_cast<InnerModelDifferentialRobot *>(*it);
			if (differentialNode != NULL)
			{
				//qDebug() << "DifferentialRobot " << differentialNode->id << differentialNode->port;
				addDFR(differentialNode);
			}
			
			InnerModelIMU* imuNode = dynamic_cast<InnerModelIMU *>(*it);
			if (imuNode != NULL)
			{
				//qDebug() << "IMU " << imuNode->id << imuNode->port;
				addIMU(imuNode);
			}
			
			InnerModelJoint* jointNode = dynamic_cast<InnerModelJoint *>(*it);
			if (jointNode != NULL)
			{
				//qDebug() << "Joint " << (*it)->id;
				addJM(jointNode);
			}

			InnerModelTouchSensor* touchNode = dynamic_cast<InnerModelTouchSensor *>(*it);
			if (touchNode != NULL)
			{
				qDebug() << "Touch " << (*it)->id << "CALLING addTouch";
				addTouch(touchNode);
			}

			walkTree(*it);
		}
	}
	
};



// ------------------------------------------------------------------------------------------------
//
// ------------------------------------------------------------------------------------------------

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker (MapPrx& _mprx, Ice::CommunicatorPtr _communicator, const char* _innerModelXML, int ms) : GenericWorker (_mprx)
{
	d = new Data;
	
	// Create the server handlers
	d->worker = this;
	d->communicator = _communicator;
	
	d->laserDataCartArray.clear();
	
	// Initialize Inner model
	d->innerModel = new InnerModel(_innerModelXML);
	//add name of .xml
	setWindowTitle(windowTitle() + "\t" + _innerModelXML);
	
	// Initialize the Inner Model Viewer
	QGLFormat fmt;
	fmt.setDoubleBuffer (true);
	QGLFormat::setDefaultFormat (fmt);
	d->viewer = new OsgView (frameOSG);
	d->imv = new InnerModelViewer (d->innerModel, "root", d->viewer->getRootGroup());
	d->manipulator = new osgGA::TrackballManipulator;
	d->manipulator->computeHomePosition(d->viewer->getCamera(),false);
	d->viewer->setCameraManipulator (d->manipulator,false);	
	
	
	settings = new QSettings ("RoboComp", "RCIS");
	QString path (_innerModelXML);
	if (path == settings->value("path").toString() ) 
	{
	  //restore matrix view 
	  QStringList l = settings->value("matrix").toStringList();
	  osg::Matrixd m;
	  for (int i=0; i<4; i++ )
	  {
	    for (int j=0; j<4; j++ )
	    {
  // 	    cout<< m(i,j)<<" ";
// 	      l.append(s.number(m(i,j)));
	      m(i,j)=l.takeFirst().toDouble();
	      }
  // 	    cout<<"\n";
	  }
	  
	  std::cout<<m;
	
	  d->manipulator->setByMatrix(m);
	
	}
	else
	{
	  settings->setValue("path",path);
	  qDebug()<<"new path"<<path<<"\n";
	}
	
	// Connect all the signals
	connect (topView,   SIGNAL (clicked()), this, SLOT (setTopPOV()));
	connect (leftView,  SIGNAL (clicked()), this, SLOT (setLeftPOV()));
	connect (rightView, SIGNAL (clicked()), this, SLOT (setRightPOV()));
	connect (frontView, SIGNAL (clicked()), this, SLOT (setFrontPOV()));
	connect (backView,  SIGNAL (clicked()), this, SLOT (setBackPOV()));
	connect (sp_lightx,  SIGNAL (valueChanged(double)), this, SLOT (setLigthx(double)));
	connect (sp_lighty,  SIGNAL (valueChanged(double)), this, SLOT (setLigthy(double)));
	connect (sp_lightz,  SIGNAL (valueChanged(double)), this, SLOT (setLigthz(double)));
	
	connect (actionObject, SIGNAL (triggered()), this, SLOT (objectTriggered()));
	connect (actionVisual, SIGNAL (triggered()), this, SLOT (visualTriggered()));
	
	objectTriggered();
	visualTriggered();
	
// 	d->innerModel->treePrint();
	
	// Start the physics simulation
	#ifdef USE_BULLET
	d->world = new World();
	//d->startPhysics();
	#endif

	// Initialize the timer
	setPeriod(ms);
	
}


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}


osg::Group *SpecificWorker::getRootGroup()
{
	return d->viewer->getRootGroup();
}


InnerModel *SpecificWorker::getInnerModel()
{
	return d->innerModel;
}


InnerModelViewer *SpecificWorker::getInnerModelViewer()
{
	return d->imv;
}


void SpecificWorker::startServers()
{
	d->walkTree();
	d->includeLasers();
	d->includeRGBDs();
}


void SpecificWorker::scheduleShutdown(JointMotorServer *j)
{
	d->jointServersToShutDown.push_back(j);
}



// ------------------------------------------------------------------------------------------------
// Slots
// ------------------------------------------------------------------------------------------------

void SpecificWorker::compute()
{
	// Compute the elapsed time interval since the last update
	static QTime lastTime = QTime::currentTime();
	QTime currentTime = QTime::currentTime();
	const int elapsed = lastTime.msecsTo (currentTime);
	lastTime = currentTime;
	
	QMutexLocker locker (mutex);
	
	
	// Remove previous laser shapes
	for (QHash<QString, IMVLaser>::iterator laser = d->imv->lasers.begin(); laser != d->imv->lasers.end(); laser++)
	{
		if (laser->osgNode->getNumChildren() > 0)
		{
			laser->osgNode->removeChild (0, laser->osgNode->getNumChildren());
		}
	}
	
	// Camera render
	QHash<QString, IMVCamera>::const_iterator i = d->imv->cameras.constBegin();
	while (i != d->imv->cameras.constEnd())
	{
		RTMat rt= d->innerModel->getTransformationMatrix ("root",i.key());
		///Put camera in her position.
		d->imv->cameras[i.key()].viewerCamera->getCameraManipulator()->setByMatrix (QMatToOSGMat4 (rt));

		for (int n=0; n<d->imv->cameras.size() ; ++n)
		{
			d->imv->cameras[i.key()].viewerCamera->frame();
		}
		i++;
	}

	// Laser rendering
	for (QHash<QString, IMVLaser>::iterator laser = d->imv->lasers.begin(); laser != d->imv->lasers.end(); laser++)
	{
		QString id=laser->laserNode->id;

		if (d->laserDataCartArray.contains (id) ==false)
		{
			//laserDataCartArray.insert(id);
			osg::Vec3Array *v= new osg::Vec3Array();
			v->resize (laser->laserNode->measures+1);
			d->laserDataCartArray.insert (id,v);
		}

		// create and insert laser data
		d->laserDataArray.insert(laser->laserNode->id, d->LASER_createLaserData (laser.value()));
		
		// create and insert laser shape
		osg::ref_ptr<osg::Node> p=NULL;
		if (id=="laserSecurity")
		{
			p = d->viewer->addPolygon (* (d->laserDataCartArray[id]), osg::Vec4 (0.,0.,1.,0.4));
		}
		else
		{
			p = d->viewer->addPolygon (* (d->laserDataCartArray[id]));
		}
		if (p!=NULL)
		{
			laser->osgNode->addChild (p);
		}
	}


#ifdef INNERMODELMANAGERDEBUG
	printf ("Elapsed time: %d\n", elapsed);
#endif
	// Update joints and compute physic interactions
	d->updateJoints(float(elapsed)/1000.0f);
	
	// Update touch sensors
	d->updateTouchSensors();
	
	
	// Shutdown empty servers
	for (int i=0; i<d->jointServersToShutDown.size(); i++)
	{
		d->jointServersToShutDown[i]->shutdown();
	}
	d->jointServersToShutDown.clear();
	
	// Resize world widget if necessary, and render the world
	if (d->viewer->size() != frameOSG->size())
	{
		d->viewer->setFixedSize (frameOSG->width(), frameOSG->height());
	}
	
	
	d->innerModel->update();
	d->imv->update();	
#ifdef USE_BULLET
	d->world->update(elapsed);
#endif
	//osg render
	d->viewer->frame();
	
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


void SpecificWorker::setTopPOV()
{
	d->imv->setMainCamera (d->manipulator, InnerModelViewer::TOP_POV);
}


void SpecificWorker::setFrontPOV()
{
	d->imv->setMainCamera (d->manipulator, InnerModelViewer::FRONT_POV);
}


void SpecificWorker::setBackPOV()
{
	d->imv->setMainCamera (d->manipulator, InnerModelViewer::BACK_POV);
}


void SpecificWorker::setLeftPOV()
{
	d->imv->setMainCamera (d->manipulator, InnerModelViewer::LEFT_POV);
}


void SpecificWorker::setRightPOV()
{
	d->imv->setMainCamera (d->manipulator, InnerModelViewer::RIGHT_POV);
}


void SpecificWorker::closeEvent (QCloseEvent *event)
{
	event->accept();
	std::cout<<d->manipulator->getMatrix();	
	osg::Matrixd m = d->manipulator->getMatrix();	
	QString s="";
	QStringList l;
	for (int i=0; i<4; i++ )
	{
	  for (int j=0; j<4; j++ )
	  {
// 	    cout<< m(i,j)<<" ";
	    l.append(s.number(m(i,j)));
	    }
// 	    cout<<"\n";
	}
	
// 	l.removeLast();
	qDebug()<<"L"<<l;
	
	settings->setValue("matrix", l);	
	qDebug()<<"toStringList"<< settings->value("matrix").toStringList();
	settings->sync();
	qDebug()<<settings->allKeys();
	
	exit (EXIT_SUCCESS);
}

void SpecificWorker::setLigthx(double v)
{
	osg::Vec4 p= d->viewer->getLight()->getPosition();	
	p.set(v,p.y(),p.z(),p.w());
	d->viewer->getLight()->setPosition(p);	
}

void SpecificWorker::setLigthy(double v)
{
	osg::Vec4 p= d->viewer->getLight()->getPosition();	
	p.set(p.x(),v,p.z(),p.w());
	d->viewer->getLight()->setPosition(p);	
}

void SpecificWorker::setLigthz(double v)
{
	osg::Vec4 p= d->viewer->getLight()->getPosition();	
	p.set(p.x(),p.y(),v,p.w());
	d->viewer->getLight()->setPosition(p);	
}




// ------------------------------------------------------------------------------------------------
// ICE interfaces
// ------------------------------------------------------------------------------------------------

// void SpecificWorker::checkPoseCollision (QString node,QString msg)
//
//{
// 	///por cada mesh descendiente chequear colisiones con todo el mundo menos con sus mesh hermanas
// #ifdef INNERMODELMANAGERDEBUG
// 	qDebug() <<"checkPoseCollision"<<msg<<node<<"Are you crazy?";
// #endif
// 	QStringList l;
// 	l.clear();
// 
// 	innerModel->getSubTree (innerModel->getNode (node),&l);
// 
// 	/// Checking
// 	foreach (QString n, l)
// 	{
// 		/// Replicate plane removals
// 		if (imv->meshHash.contains (n))
// 		{
// 			QList <QString> excludingList;
// 			excludingList.clear();
// 			detectarColision1toN (n,excludingList,msg);
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


#include "simulator/specificworker_cam.h"
#include "simulator/specificworker_cm.h"
#include "simulator/specificworker_dfr.h"
#include "simulator/specificworker_imm.h"
#include "simulator/specificworker_imu.h"
#include "simulator/specificworker_jm.h"
#include "simulator/specificworker_laser.h"
#include "simulator/specificworker_rgbd.h"
