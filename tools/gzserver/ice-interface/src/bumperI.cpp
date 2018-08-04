#include "bumperI.h"

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include <gazebo/gazebo_config.h>

using namespace RoboCompBumper;
using namespace std;
using namespace gazebo; 

bumperI::bumperI(int argc, char **argv) {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
#else
    gazebo::client::setup(argc, argv);
#endif

    this->device_name_ = "gazebo_robocomp_bumper";
    this->topic_name_ = "";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(topic_name_, &bumperI::callback, this);
} 

bumperI::~bumperI() {
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
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

