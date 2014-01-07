#pragma once

#include "cameraI.h"
#include "differentialrobotI.h"
#include "imuI.h"
#include "jointmotorI.h"
#include "laserI.h"
#include "rgbdI.h"

#include <CommonHead.h>

// Namespaces
// using namespace std;
// using namespace RoboCompCamera;
// using namespace RoboCompCommonHead;
// using namespace RoboCompDifferentialRobot;
// using namespace RoboCompLaser;
// using namespace RoboCompIMU;

/** XXXServer **/
/** XXXServer **/
/** XXXServer **/
class JointMotorServer
{
public:
	JointMotorServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker_, uint32_t _port);
	void add(IM2::HingeJoint *joint);
	void remove(IM2::HingeJoint *joint);
	bool empty();
	void shutdown();

	uint32_t port;
	Ice::CommunicatorPtr comm;
	Ice::ObjectAdapterPtr adapter;
	JointMotorI *interface;
	std::vector<IM2::HingeJoint *> joints;
	SpecificWorker *worker;
};


class LaserServer
{
public:
	LaserServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(IM2::Laser *laser);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	LaserI *interface;
	std::vector<IM2::Laser *> lasers;
};


class RGBDServer
{
public:
	RGBDServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(IM2::RGBD *rgbd);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	RGBDI *interface;
	std::vector<IM2::RGBD *> rgbds;
};


class IMUServer
{
public:
	IMUServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(IM2::IMU *imu);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	IMUI *interface;
	std::vector<IM2::IMU *> imus;
};


class DifferentialRobotServer
{
public:
	DifferentialRobotServer(Ice::CommunicatorPtr communicator, SpecificWorker *worker, uint32_t _port);
	void add(IM2::DifferentialRobot *differentialrobot);

	uint32_t port;
	Ice::ObjectAdapterPtr adapter;
	DifferentialRobotI *interface;
	std::vector<IM2::DifferentialRobot *> differentialrobots;
};
