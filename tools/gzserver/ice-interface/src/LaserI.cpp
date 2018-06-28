#include "LaserI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

using namespace RoboCompLaser;
using namespace std;
using namespace gazebo; 

#include "raysensor.pb.h"
#include "Laser_msgs.pb.h"

typedef const boost::shared_ptr<const Laser_msgs::msgs::gazebo_robocomp_laser> ConstGazeboRoboCompLaserPtr;

LaserI::LaserI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_laser";
    this->topic_name_ = "/gazebo_robocomp_laser/data";
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

void LaserI::callback(ConstGazeboRoboCompLaserPtr &_msg) {

    int i = 0;
    std::cerr << "Getting Callbacks" << std::endl;
    LaserScanValues.resize(_msg->range_size());

    for(i = 0; i < _msg->range_size(); i++)
    {
        this->LaserScanValues[i].angle = _msg->laser().horizontal_resolution()*i + _msg->laser().horizontal_min_angle();
        this->LaserScanValues[i].dist = _msg->range(i);
    }
    
    if (this->seed_ < 2) {
        this->LaserConfigData.maxMeasures = _msg->laser().horizontal_samples();
        this->LaserConfigData.maxDegrees = _msg->laser().horizontal_max_angle();
        this->LaserConfigData.maxRange = _msg->laser().range_max();
        this->LaserConfigData.minRange = _msg->laser().range_min();
        this->LaserConfigData.iniRange = _msg->range(0);
        this->LaserConfigData.endRange = _msg->range(_msg->range_size()-1);
        this->LaserConfigData.angleRes = _msg->laser().horizontal_resolution();
        this->LaserConfigData.angleIni = _msg->laser().horizontal_min_angle();
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

