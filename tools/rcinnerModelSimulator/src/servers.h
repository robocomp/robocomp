#ifndef SERVER_H
#define SERVER_H
#pragma once

#include "cameraI.h"
#include "differentialrobotI.h"
#include "omnirobotI.h"
#include "imuI.h"
#include "jointmotorI.h"
#include "laserI.h"
#include "rgbdI.h"
#include "touchsensorI.h"
#include "genericbaseI.h"
#include "displayI.h"

#include <CommonHead.h>


// -----------------------------------------------------------
// JointMotorServer
// -----------------------------------------------------------
// class JointMotorServer
// {
// public:
// 	JointMotorServer(Ice::CommunicatorPtr communicator, std::shared_ptr<SpecificWorker> worker_, uint32_t _port);
// 	void add(InnerModelJoint *joint);
// 	void remove(InnerModelJoint *joint);
// 	void add(InnerModelPrismaticJoint *joint);
// 	void remove(InnerModelPrismaticJoint *joint);
// 	bool empty();
// 	void shutdown();
// 
// 	uint32_t port;
// 	Ice::CommunicatorPtr comm;
// 	Ice::ObjectAdapterPtr adapter;
// 	JointMotorI *interface;
// 	std::vector<void *> joints;
// 	std::shared_ptr<SpecificWorker> worker;
// };

// -----------------------------------------------------------
// DisplayServer
// -----------------------------------------------------------
// class DisplayServer
// {
// public:
// 	DisplayServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker_, uint32_t _port);
// 	void add(InnerModelDisplay *display);
// 	void remove(InnerModelDisplay *display);
// 	bool empty();
// 	void shutdown();
// 
// 	uint32_t port;
// 	Ice::CommunicatorPtr comm;
// 	Ice::ObjectAdapterPtr adapter;
// 	DisplayI *interface;
// 	std::shared_ptr<SpecificWorker> worker;
// };

// -----------------------------------------------------------
// TouchSensorServer
// -----------------------------------------------------------
// class TouchSensorServer
// {
// public:
// 	TouchSensorServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker_, uint32_t _port);
// 	void add(InnerModelTouchSensor *sensor);
// 	void remove(InnerModelTouchSensor *sensor);
// 	bool empty();
// 	void shutdown();
// 
// 	uint32_t port;
// 	Ice::CommunicatorPtr comm;
// 	Ice::ObjectAdapterPtr adapter;
// 	TouchSensorI *interface;
// 	std::vector<InnerModelTouchSensor *> sensors;
// 	std::shared_ptr<SpecificWorker> worker;
// };

// -----------------------------------------------------------
// LaserServer
// // -----------------------------------------------------------
// class LaserServer
// {
// public:
// 	LaserServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
// 	void add(InnerModelLaser *laser);
// 
// 	uint32_t port;
// 	Ice::ObjectAdapterPtr adapter;
// 	LaserI *interface;
// 	std::vector<InnerModelLaser *> lasers;
// };

// -----------------------------------------------------------
// RGBDServer
// -----------------------------------------------------------
// class RGBDServer
// {
// public:
// 	RGBDServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
// 	void add(InnerModelRGBD *rgbd);
// 
// 	uint32_t port;
// 	Ice::ObjectAdapterPtr adapter;
// 	RGBDI *interface;
// 	std::vector<InnerModelRGBD *> rgbds;
// };

// -----------------------------------------------------------
// IMUServer
// // -----------------------------------------------------------
// class IMUServer
// {
// public:
// 	IMUServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
// 	void add(InnerModelIMU *imu);
// 
// 	uint32_t port;
// 	Ice::ObjectAdapterPtr adapter;
// 	IMUI *interface;
// 	std::vector<InnerModelIMU *> imus;
// };

// -----------------------------------------------------------
// DifferentialRobotServer
// -----------------------------------------------------------
// class DifferentialRobotServer
// {
// public:
// 	DifferentialRobotServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
// 	void add(InnerModelDifferentialRobot *differentialrobot);
// 
// 	uint32_t port;
// 	Ice::ObjectAdapterPtr adapter;
// 	DifferentialRobotI *interface;
// 	std::vector<InnerModelDifferentialRobot *> differentialrobots;
// 	std::string name = "DifferentialRobotServer";
// };

// -----------------------------------------------------------
// OmniRobotServer
// -----------------------------------------------------------
// class OmniRobotServer
// {
// public:
// 	OmniRobotServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
// 	void add(InnerModelOmniRobot *omnirobot);
// 
// 	uint32_t port;
// 	Ice::ObjectAdapterPtr adapter;
// 	OmniRobotI *interface;
// 	DifferentialRobotI *interfaceDFR;
// 	GenericBaseI *interfaceGB;
// 	std::vector<InnerModelOmniRobot *> omnirobots;
// };
#endif
