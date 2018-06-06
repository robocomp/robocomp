#include "bumperI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

using namespace RoboCompBumper;
using namespace std;
using namespace gazebo; 

bumperI::bumperI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_bumper";
    this->topic_name_ = "";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(topic_name_, &bumperI::callback, this);
} 

bumperI::~bumperI() {
    gazebo::client::shutdown();
}

void bumperI::enable(const Ice::Current&) {

}

void bumperI::disable(const Ice::Current&) {

}

bool bumperI::isEnabled(const Ice::Current&) {

}

void bumperI::reset(const Ice::Current&) {

}

SensorStateMap bumperI::getSensorData(const Ice::Current&) {

}

void bumperI::callback(ConstContactsPtr &_msg) {

} 

