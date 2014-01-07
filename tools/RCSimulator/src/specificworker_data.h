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

#pragma once

// Simulator includes
#include "specificworker.h"

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

// RoboComp includes
#include <osgviewer/osgview.h>

// #define INNERMODELMANAGERDEBUG


struct JointMovement {
	float endPos;
	float endSpeed;
	float maxAcc;
	enum { FixedPosition, TargetPosition, TargetSpeed } mode;
};



struct SpecificWorker::Data {
	// World
	SpecificWorker* worker;
	IM2::InnerModel* innerModel;
	IM2::Viewer* imv;
	OsgView* viewer;
	osgGA::TrackballManipulator* manipulator;
	
	// Handlers
	Ice::CommunicatorPtr communicator;
	std::map<uint32_t, JointMotorServer> jm_servers;
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
	
	RoboCompLaser::TLaserData LASER_createLaserData ( const IM2::Laser &laser );
	IM2::Node *getNode ( const QString &id, const QString &msg );
	void checkOperationInvalidNode ( IM2::Node *node,QString msg );
	void checkNodeAlreadyExists ( const QString &id, const QString &msg );
	void checkInvalidMeshValues ( RoboCompInnerModelManager::meshType m, QString msg );
	void AttributeAlreadyExists ( IM2::Node* node, QString attributeName, QString msg );
	void NonExistingAttribute ( IM2::Node* node, QString attributeName, QString msg );
	void getRecursiveNodeInformation ( RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, IM2::Node* node );
	RoboCompInnerModelManager::NodeType getNodeType ( IM2::Node* node );

	// ------------------------------------------------------------------------------------------------
	// Private methods
	// ------------------------------------------------------------------------------------------------
	
	void updateJoints( const float delta );
	void addDFR ( IM2::DifferentialRobot* node );
	void addIMU ( IM2::IMU* node );
	void addJM( IM2::HingeJoint* node );
	void addLaser ( IM2::Laser* node );
	void addRGBD ( IM2::RGBD* node );
	void removeJM( IM2::HingeJoint* node );
	void walkTree( IM2::Node* node = NULL );
	void scheduleShutdown( JointMotorServer *j );
};
