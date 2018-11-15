/*
 *    Copyright (C) 2006-2014 by RoboLab - University of Extremadura
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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Qt includes
#if Qt5_FOUND 
	#include <QtOpenGL>
	#include <QGLWidget>
#endif
#include <QtCore>
#include <QtGui>
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

#include <memory>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include <InnerModelManager.h>
#include <osgviewer/osgview.h>

#include "genericworker.h"
#include "pickhandler.h"
#include "serversinitiator.h"
#include <fps/fps.h>

using guard = std::lock_guard<std::recursive_mutex>;	
using InnerPtr = std::shared_ptr<InnerModel>;

class SpecificWorker : public GenericWorker
{
		
Q_OBJECT
	public:
		SpecificWorker(MapPrx &_mprx, Ice::CommunicatorPtr _communicator, const char *_innerModelXML, int ms);
		~SpecificWorker(){};
		osg::Group *getRootGroup();
		std::shared_ptr<InnerModel> getInnerModel();
		std::shared_ptr<InnerModelViewer> getInnerModelViewer();
		
		std::shared_ptr<InnerModel> innerModel;
		std::shared_ptr<InnerModelViewer> imv;
		std::shared_ptr<SpecificWorker> worker{ this };
	
		// Handlers
		ServersInitiator servers;
		Ice::CommunicatorPtr communicator;
		
		// Laser
		std::map<std::string, RoboCompLaser::TLaserData> laserDataArray;
		std::map<std::string, osg::Vec3Array*> laserDataCartArray;
		
		// JointMotor
		struct JointMovement
		{
			float endPos;
			float endSpeed;
			float maxAcc;
			enum { FixedPosition, TargetPosition, TargetSpeed } mode;
		};
		QHash<QString, JointMovement> jointMovements;
		
	private:
		QSettings *settings;
		OsgView *viewer;
		osgGA::TrackballManipulator *manipulator;
		
		//RoboCompLaser::TLaserData LASER_createLaserData(const IMVLaser &laser);
		//RoboCompTouchSensor::SensorMap TOUCH_createTouchData(const IMVLaser &laser);
		
		FPSCounter fps;
		
	public:
		InnerModelNode *getNode(const QString &id, const QString &msg);
		void checkOperationInvalidNode(InnerModelNode *node,QString msg);
		void checkNodeAlreadyExists(const QString &id, const QString &msg);
		void checkInvalidMeshValues(RoboCompInnerModelManager::meshType m, QString msg);
		void AttributeAlreadyExists(InnerModelNode *node, QString attributeName, QString msg);
		void NonExistingAttribute(InnerModelNode *node, QString attributeName, QString msg);
		void getRecursiveNodeInformation(RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, InnerModelNode *node);
		RoboCompInnerModelManager::NodeType getNodeType(InnerModelNode *node);
	
  private:
		void cambiaColor(QString id, osg::Vec4 color);
		void devuelveColor(QString id);
		void changeLigthState(bool apagar);
		void updateJoints(const float delta);
		void updateTouchSensors();
		void updateCameras();
		void updateLasers();
	
	public slots:
		// ----------------------------------------------------------------------------------------
		// GUI event handlers
		// ----------------------------------------------------------------------------------------
		void compute();
		void objectTriggered();
		void visualTriggered();
		void setTopPOV();
		void setFrontPOV();
		void setBackPOV();
		void setLeftPOV();
		void setRightPOV();
		void setLigthx(double v);
		void setLigthy(double v);
		void setLigthz(double v);
		void closeEvent(QCloseEvent *event);
};

#endif
