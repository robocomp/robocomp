#include "MotorI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

typedef const boost::shared_ptr<const joint_motor_state::msgs::JointMotorState> ConstJointMotorStatePtr;
typedef const boost::shared_ptr<const motor_goal_vel::msgs::MotorGoalVelocity> ConstMotorGoalVelocityPtr;
typedef const boost::shared_ptr<const motor_goal_position::msgs::MotorGoalPosition> ConstMotorGoalPositionPtr;

using namespace RoboCompMotors;
using namespace std;
using namespace gazebo;

MotorI::MotorI(int argc, char **argv) {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
#else
    gazebo::client::setup(argc, argv);
#endif    
    this->sub_topic_name_ = "/gazebo/joint/vel";
    this->vel_topic_name_ = "/speed/cmd";
    this->pos_topic_name_ = "/position/cmd";

    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();

    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &MotorI::callback, this);
    this->pos_pub_ = this->gazebo_node_->Advertise<motor_goal_position::msgs::MotorGoalPosition>(pos_topic_name_);
    this->vel_pub_ = this->gazebo_node_->Advertise<motor_goal_vel::msgs::MotorGoalVelocity>(vel_topic_name_);
} 

MotorI::~MotorI() {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}

void MotorI::setPosition(const MotorGoalPosition& goal, const Ice::Current&) {
    motor_goal_position::msgs::MotorGoalPosition msg;
    msg.set_position(goal.position);
    msg.set_maxvel(goal.maxSpeed);
    msg.set_name(goal.name);

    pos_pub_->Publish(msg);
}

void MotorI::setVelocity(const MotorGoalVelocity& goal, const Ice::Current&) {
    motor_goal_vel::msgs::MotorGoalVelocity msg;
    msg.set_velocity(goal.velocity);
    msg.set_maxaccel(goal.maxAcc);
    msg.set_name(goal.name);
    vel_pub_->Publish(msg);
}

MotorState MotorI::getState(const Ice::Current&) {
    this->motor_state_;
}

void MotorI::setZeroPos(const Ice::Current&) {
    motor_goal_position::msgs::MotorGoalPosition msg;
    msg.set_position(0);
    msg.set_maxvel(10);
    msg.set_name("");
    pos_pub_->Publish(msg);
}

void MotorI::callback(ConstJointMotorStatePtr &_msg) {
    this->motor_state_.p = 0;
    this->motor_state_.v = 0;
    this->motor_state_.temperature = 0;
    this->motor_state_.isMoving = true;
    this->motor_state_.pos = _msg->position();
    this->motor_state_.vel = _msg->speed();
    this->motor_state_.power = _msg->torque()*_msg->speed();
    this->motor_state_.timeStamp = _msg->timestamp();
}
