#include "LaserI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

using namespace RoboCompLaser;
using namespace std;
using namespace gazebo; 

LaserI::LaserI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_laser";
    this->topic_name_ = "/gazebo/gazebo_robocomp_laser/hokuyo/hokuyo/link/laser/scan";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(topic_name_, &LaserI::callback, this);
    this->seed_ = 0;
    
    int i = 0;

    LaserScanValues.resize(10);
    for(i = 0; i < 10; i++)
    {
        this->LaserScanValues[i].angle = 0;
        this->LaserScanValues[i].dist = 0;
    }
    
    this->LaserConfigData.maxMeasures = 0;
    this->LaserConfigData.maxDegrees = 0;
    this->LaserConfigData.maxRange = 0;
    this->LaserConfigData.minRange = 0;
    this->LaserConfigData.iniRange = 0; 
    this->LaserConfigData.endRange = 0;
    this->LaserConfigData.angleRes = 0;
    this->LaserConfigData.angleIni = 0; 
    this->LaserConfigData.device = "gazebo_robocomp_laser";
    this->LaserConfigData.driver = "gazebo_driver";
    this->LaserConfigData.sampleRate = 0;
    this->LaserConfigData.staticConf = 0;
    this->LaserConfigData.cluster = 0;
} 

LaserI::~LaserI() {
    gazebo::client::shutdown();
}

TLaserData LaserI::getLaserData(const Ice::Current&) {
    std::cerr << "Returning LaserData" << std::endl;
    return LaserScanValues;
}

LaserConfData LaserI::getLaserConfData(const Ice::Current&) {
    std::cerr << "Returning Config Data" << std::endl;
    return LaserConfigData;
}

void LaserI::callback(ConstLaserScanStampedPtr &_msg) {

    int i = 0;
    std::cerr << "Getting Callbacks" << std::endl;
    LaserScanValues.resize(_msg->scan().count());

    for(i = 0; i < _msg->scan().count(); i++)
    {
        this->LaserScanValues[i].angle = _msg->scan().angle_step()*i + _msg->scan().angle_min();
        this->LaserScanValues[i].dist = _msg->scan().ranges(i);
    }
    
    if (this->seed_ < 2) {
        this->LaserConfigData.maxMeasures = _msg->scan().count();
        this->LaserConfigData.maxDegrees = _msg->scan().angle_max();
        this->LaserConfigData.maxRange = _msg->scan().range_min();
        this->LaserConfigData.minRange = _msg->scan().range_max();
        this->LaserConfigData.iniRange = _msg->scan().ranges(0);
        this->LaserConfigData.endRange = _msg->scan().ranges(i-1);
        this->LaserConfigData.angleRes = _msg->scan().angle_step();
        this->LaserConfigData.angleIni = _msg->scan().angle_min();
        this->LaserConfigData.device = this->device_name_;
        this->LaserConfigData.driver = "gazebo_driver";
        this->LaserConfigData.sampleRate = 0;
        this->LaserConfigData.staticConf = 0;
        this->LaserConfigData.cluster = 0;
        this->seed_++;
    }
} 

TLaserData LaserI::getLaserAndBStateData(RoboCompGenericBase::TBaseState& bState, const Ice::Current&) 
{
    return LaserScanValues;
}

