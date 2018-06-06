#include "MotorI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

using namespace RoboCompMotors;
using namespace std;
using namespace gazebo;

MotorI::MotorI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_motor";
    this->motor_speed_ = 0;
    this->sub_topic_name_ = "/gazebo/joint/vel";
    this->pub_topic_name_ = "/speed/cmd";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &MotorI::callback, this);
    this->pub_ = this->gazebo_node_->Advertise<msgs::Vector3d>(pub_topic_name_);
} 

MotorI::~MotorI() {
    gazebo::client::shutdown();
}

void MotorI::setMotorSpeed(Ice::Float &angVel, const Ice::Current&) {
    msgs::Vector3d msg;
    pub_->WaitForConnection();
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(angVel, 0, 0));
    pub_->Publish(msg);
    std::cerr << "Setting Angular velocity of the joint to " << angVel << std::endl;
}

float MotorI::getMotorSpeed(const Ice::Current&) {
    return this->motor_speed_;
}

void MotorI::callback(ConstVector3dPtr &_msg) {
    this->motor_speed_ = _msg->x();
} 


