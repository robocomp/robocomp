#include "DifferentialRobotI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
 
using namespace RoboCompDifferentialRobot;
using namespace std;
using namespace gazebo; 
using namespace diffdrive_cmd::msgs; 
using namespace diffdrive_state::msgs;

typedef const boost::shared_ptr<const diffdrive_cmd::msgs::DiffDriveCmd> DiffDriveCmdPtr;
typedef const boost::shared_ptr<const diffdrive_state::msgs::DiffDriveState> DiffDriveStatePtr;

DifferentialRobotI::DifferentialRobotI(int argc, char **argv) {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
#else
    gazebo::client::setup(argc, argv);
#endif
    this->device_name_ = "gazebo_robocomp_diffdrive";
    this->sub_topic_name_ = "/diffdrive/data";
    this->pub_topic_name_ = "/my_robot";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &DifferentialRobotI::callback, this);
    this->pub_ = this->gazebo_node_->Advertise<DiffDriveCmd>(pub_topic_name_);
} 

DifferentialRobotI::~DifferentialRobotI() {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}

void DifferentialRobotI::getBaseState(RoboCompGenericBase::TBaseState& _base_state, const Ice::Current&) {
    // store the config value of base in _base_state
    _base_state = state_;
} 
void DifferentialRobotI::getBasePose(int& x, int& z, float& alpha, const Ice::Current&) {
    // listen to the topic where the model pose is being published
    x = state_.x;
    z = state_.z;
    alpha = state_.alpha;
}
void DifferentialRobotI::setSpeedBase(float adv, float rot, const Ice::Current&) {
    // declare a vector3d msg and set the adv and rot into that and publish it over a topic where the plugin
    DiffDriveCmd cmd_;
    cmd_.set_linear_vel(adv);
    cmd_.set_angular_vel(rot);

    pub_->Publish(cmd_);
}
void DifferentialRobotI::stopBase(const Ice::Current&) {
    // same as above just with both values being set to zero
    DiffDriveCmd cmd_;
    cmd_.set_linear_vel(0);
    cmd_.set_angular_vel(0);
    pub_->Publish(cmd_);

}
void DifferentialRobotI::resetOdometer(const Ice::Current&) {

}
void DifferentialRobotI::setOdometer(const RoboCompGenericBase::TBaseState& _base_state, const Ice::Current&) {

}
void DifferentialRobotI::setOdometerPose(int x, int z, float alpha, const Ice::Current&) {

}
void DifferentialRobotI::correctOdometer(int x, int z, float alpha, const Ice::Current&) {

}

void DifferentialRobotI::callback(DiffDriveStatePtr &_msg) {

    if (_msg->linvel().x()||_msg->linvel().x()||_msg->linvel().x()) this->state_.isMoving = true;
    this->state_.x = _msg->pose().position().x();
    this->state_.z = _msg->pose().position().z();
    this->state_.alpha = _msg->pose().orientation().y();
    this->state_.correctedX = 0;
    this->state_.correctedZ = 0;
    this->state_.correctedAlpha = 0;
    this->state_.advVx = _msg->linvel().x();
    this->state_.advVz = _msg->linvel().z();
    this->state_.rotV = _msg->angvel().y();

}