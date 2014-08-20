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
#include <innermodel/innermodelviewer.h>

#include "genericworker.h"
#include "servers.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	struct Data;
	Data* d;
	QSettings *settings;

public:
	SpecificWorker(MapPrx &_mprx, Ice::CommunicatorPtr _communicator, const char *_innerModelXML, int ms);
	~SpecificWorker();

	osg::Group *getRootGroup();
	InnerModel *getInnerModel();
	InnerModelViewer* getInnerModelViewer();
	QMutex *viewerMutex;
	void startServers();
	void scheduleShutdown(JointMotorServer *j);
	
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
	void cam_camera_getYUVImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void cam_getYImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void cam_getYLogPolarImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void cam_getYImageCR(const QString& server, int cam, int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void cam_getRGBPackedImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void cam_getYRGBImage(const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState);
	TCamParams cam_getCamParams(const QString& server);
	void cam_setInnerImage(const QString& server, const RoboCompCamera::imgType& roi);
	
	// ----------------------------------------------------------------------------------------
	// CommonBehavior.ice
	// ----------------------------------------------------------------------------------------
	
	// ----------------------------------------------------------------------------------------
	// DifferentialRobot.ice
	// ----------------------------------------------------------------------------------------
	void dfr_getBaseState(const QString& server, RoboCompDifferentialRobot::TBaseState& state);
	void dfr_getBasePose(const QString& server, int& x, int& z, float& alpha);
	void dfr_setSpeedBase(const QString& server, float adv, float rot);
	void dfr_stopBase(const QString& server);
	void dfr_resetOdometer(const QString& server);
	void dfr_setOdometer(const QString& server, const RoboCompDifferentialRobot::TBaseState& state);
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
	TLaserData laser_getLaserAndBStateData(const QString& server, RoboCompDifferentialRobot::TBaseState& state);
	LaserConfData laser_getLaserConfData(const QString& server);
	
	// ----------------------------------------------------------------------------------------
	// RGBD.ice
	// ----------------------------------------------------------------------------------------
	TRGBDParams rgbd_getRGBDParams(const QString& server);
	void rgbd_setRegistration(const QString& server, Registration value);
	Registration rgbd_getRegistration(const QString& server);
	void rgbd_getData(const QString& server, RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState);
	void rgbd_getImage(const QString& server, ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState);
};

#endif
