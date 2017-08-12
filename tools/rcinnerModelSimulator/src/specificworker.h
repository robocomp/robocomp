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
#include <QObject>
#include <QVariant>
#include <QMutex>
#include <QMutexLocker>

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <InnerModelManager.h>
#include <osgviewer/osgview.h>

#include "genericworker.h"
#include "servers.h"
#include "pickhandler.h"

#include <innermodel/innermodelmgr.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
	public:
		SpecificWorker(MapPrx &_mprx, Ice::CommunicatorPtr _communicator, const char *_innerModelXML, int ms);
		~SpecificWorker(){};
		osg::Group *getRootGroup();
		InnerModel *getInnerModel();
		InnerModelMgr getInnerModelMgr();
		InnerModelViewer* getInnerModelViewer();
		QMutex *viewerMutex;
		void startServers();
		void scheduleShutdown(JointMotorServer *j);
		
private:
		QSettings *settings;
		
		//InnerModel
		InnerModelMgr innerModel;
		
		// World
		SpecificWorker *worker;
		//InnerModel *innerModel;
		InnerModelViewer *imv;
		OsgView *viewer;
		osgGA::TrackballManipulator *manipulator;

		// Handlers
		Ice::CommunicatorPtr communicator;
		std::map<uint32_t, JointMotorServer> jm_servers;
		std::map<uint32_t, TouchSensorServer> touch_servers;
		std::map<uint32_t, LaserServer> laser_servers;
		std::map<uint32_t, RGBDServer> rgbd_servers;
		std::map<uint32_t, IMUServer> imu_servers;
		std::map<uint32_t, DifferentialRobotServer> dfr_servers;
		std::map<uint32_t, OmniRobotServer> omn_servers;

		QList <JointMotorServer *> jointServersToShutDown;
		
		// IMU
		DataImu data_imu;

		// JointMotor
		struct JointMovement
		{
			float endPos;
			float endSpeed;
			float maxAcc;
			enum { FixedPosition, TargetPosition, TargetSpeed } mode;
		};
		QHash<QString, JointMovement> jointMovements;

		// Laser
		QMap<QString, RoboCompLaser::TLaserData> laserDataArray;
		QMap<QString, osg::Vec3Array*> laserDataCartArray;
		QMutex *laserDataCartArray_mutex;
		
		// ------------------------------------------------------------------------------------------------
		// Methods
		// ------------------------------------------------------------------------------------------------
		RoboCompLaser::TLaserData LASER_createLaserData(const IMVLaser &laser);
		//RoboCompTouchSensor::SensorMap TOUCH_createTouchData(const IMVLaser &laser);
		InnerModelNode *getNode(const QString &id, const QString &msg);
		void checkOperationInvalidNode(InnerModelNode *node,QString msg);
		void checkNodeAlreadyExists(const QString &id, const QString &msg);
		void checkInvalidMeshValues(RoboCompInnerModelManager::meshType m, QString msg);
		void AttributeAlreadyExists(InnerModelNode *node, QString attributeName, QString msg);
		void NonExistingAttribute(InnerModelNode *node, QString attributeName, QString msg);
		void getRecursiveNodeInformation(RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, InnerModelNode *node);
		RoboCompInnerModelManager::NodeType getNodeType(InnerModelNode *node);
		void cambiaColor(QString id, osg::Vec4 color);
		void devuelveColor(QString id);
		void changeLigthState(bool apagar);
		
		void updateJoints(const float delta);
		void updateTouchSensors();
		void updateCameras();
		void updateLasers();
		
		void addDFR(InnerModelDifferentialRobot *node);
		void addOMN(InnerModelOmniRobot *node);
		void addIMU(InnerModelIMU *node);
		void addJM(InnerModelJoint *node);
		void addJM(InnerModelPrismaticJoint *node);
		void addTouch(InnerModelTouchSensor *node);
		void addLaser(InnerModelLaser *node);
		void addRGBD(InnerModelRGBD *node);
		void removeJM(InnerModelJoint *node);
		void includeLasers();
		void includeRGBDs();
		void walkTree(InnerModelNode *node = NULL);
	
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

	public:
		// ----------------------------------------------------------------------------------------
		// Camera.ice
		// ----------------------------------------------------------------------------------------
		void cam_camera_getYUVImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		void cam_getYImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		void cam_getYLogPolarImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		void cam_getYImageCR(const QString& server, int cam, int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		void cam_getRGBPackedImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		void cam_getYRGBImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState);
		TCamParams cam_getCamParams(const QString& server);
		void cam_setInnerImage(const QString& server, const RoboCompCamera::imgType& roi);
		
		// ----------------------------------------------------------------------------------------
		// CommonBehavior.ice
		// ----------------------------------------------------------------------------------------
		
		// ----------------------------------------------------------------------------------------
		// DifferentialRobot.ice
		// ----------------------------------------------------------------------------------------
		void dfr_getBaseState(const QString& server, RoboCompGenericBase::TBaseState& state);
		void dfr_getBasePose(const QString& server, int& x, int& z, float& alpha);
		void dfr_setSpeedBase(const QString& server, float adv, float rot);
		void dfr_stopBase(const QString& server);
		void dfr_resetOdometer(const QString& server);
		void dfr_setOdometer(const QString& server, const RoboCompGenericBase::TBaseState& state);
		void dfr_setOdometerPose(const QString& server, int x, int z, float alpha);
		void dfr_correctOdometer(const QString& server, int x, int z, float alpha);
		
		// ----------------------------------------------------------------------------------------
		// IMU.ice
		// ----------------------------------------------------------------------------------------
		void imu_updateIMUData(const QString& server, QString id);
		DataImu imu_getDataImu(const QString& server);
		void imu_resetImu(const QString& server);
		
		// ----------------------------------------------------------------------------------------
		// InnerModelManager.ice
		// ----------------------------------------------------------------------------------------
		bool imm_setPose(const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::Pose3D& pose);
		bool imm_setPoseFromParent(const QString& server, const std::string& item, const RoboCompInnerModelManager::Pose3D& pose);
		bool imm_getPose(const QString& server, const std::string& item, const std::string& base, RoboCompInnerModelManager::Pose3D& pose);
		bool imm_getPoseFromParent(const QString& server, const std::string& item, RoboCompInnerModelManager::Pose3D& pose);
		bool imm_transform(const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::coord3D& coordInItem, RoboCompInnerModelManager::coord3D& coordInBase);
		RoboCompInnerModelManager::Matrix imm_getTransformationMatrix(const std::string& item, const std::string& base);
		bool imm_setScale(const QString& server, const std::string& item, float scaleX, float scaleY, float scaleZ);
		bool imm_setPlane(const QString& server, const std::string& item, const RoboCompInnerModelManager::Plane3D& pose);
		bool imm_setPlaneTexture(const QString& server, const std::string& item, const std::string& texture);
		bool imm_addTransform(const QString& server, const std::string& item, const std::string& engine, const std::string& base, const RoboCompInnerModelManager::Pose3D& pose);
		bool imm_addJoint(const QString& server, const std::string& item,const std::string& base, const RoboCompInnerModelManager::jointType& j);
		bool imm_addMesh(const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::meshType& m);
		bool imm_addPlane(const QString& server, const std::string &item, const std::string &base, const RoboCompInnerModelManager::Plane3D &p);
		bool imm_addAttribute(const QString& server, const std::string& idNode, const std::string& name, const std::string& type, const std::string& value);
		bool imm_removeAttribute(const QString& server, const std::string& idNode, const std::string& name);
		bool imm_setAttribute(const QString& server, const std::string& idNode, const std::string& name, const std::string& type, const std::string& value);
		bool imm_getAttribute(const QString& server, const std::string& idNode, const std::string& name, std::string& type, std::string& value);
		bool imm_removeNode(const QString& server, const std::string& item);
		bool imm_moveNode(const QString& server, const std::string &src, const std::string &dst);
		void imm_getAllNodeInformation(const QString& server, RoboCompInnerModelManager::NodeInformationSequence &nodesInfo);
		void imm_setPointCloudData(const QString& server, const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud);
		bool imm_collide(const string &a, const string &b);
		
		// ----------------------------------------------------------------------------------------
		// JointMotor.ice
		// ----------------------------------------------------------------------------------------
		void jm_setPosition(const QString& server, const MotorGoalPosition& goal);
		void jm_setVelocity(const QString& server, const MotorGoalVelocity& goal);
		void jm_setSyncPosition(const QString& server, const MotorGoalPositionList& listGoals);
		void jm_setSyncVelocity(const QString& server, const MotorGoalVelocityList& listGoals);
		MotorParams jm_getMotorParams(const QString& server, const std::string& motor);
		MotorState jm_getMotorState(const QString& server, const std::string& motor);
		MotorStateMap jm_getMotorStateMap(const QString& server, const MotorList& mList);
		void jm_getAllMotorState(const QString& server, MotorStateMap& mstateMap);
		MotorParamsList jm_getAllMotorParams(const QString& server);
		RoboCompJointMotor::BusParams jm_getBusParams(const QString& server);
		void jm_setZeroPos(const QString& server, const std::string& motor);
		void jm_setSyncZeroPos(const QString& server);
		void jm_stopAllMotors(const QString& server);
		void jm_stopMotor(const QString& server, const std::string& motor);
		void jm_releaseBrakeAllMotors(const QString& server);
		void jm_releaseBrakeMotor(const QString& server, const std::string& motor);
		void jm_enableBrakeAllMotors(const QString& server);
		void jm_enableBrakeMotor(const QString& server, const std::string& motor);
		
		// ----------------------------------------------------------------------------------------
		// Laser.ice
		// ----------------------------------------------------------------------------------------
		TLaserData laser_getLaserAndBStateData(const QString& server, RoboCompGenericBase::TBaseState& state);
		LaserConfData laser_getLaserConfData(const QString& server);
		
		// ----------------------------------------------------------------------------------------
		// RGBD.ice
		// ----------------------------------------------------------------------------------------
		TRGBDParams rgbd_getRGBDParams(const QString& server);
		void rgbd_setRegistration(const QString& server, Registration value);
		Registration rgbd_getRegistration(const QString& server);
		void rgbd_getData(const QString& server, RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState);
		void rgbd_getImage(const QString& server, ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState);
};



#endif
