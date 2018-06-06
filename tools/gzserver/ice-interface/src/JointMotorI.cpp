#include "JointMotorI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

#include <jointMotor_params.pb.h>
#include <jointMotorState.pb.h>
#include <motor_goal_position.pb.h>
#include <motor_goal_velocity.pb.h>

using namespace RoboCompJointMotor;
using namespace std;
using namespace gazebo;

JointMotorI::JointMotorI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_motor";
    this->motor_speed_ = 0;

    this->sub_topic_name_ = "/gazebo/joint/state";
    this->pos_goal_topic_ = "/speed/cmd";
    this->vel_goal_topic_ = "/position/cmd";
    
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    
    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &JointMotorI::callback, this);
    this->pos_goal_pub_ = this->gazebo_node_->Advertise<motor_goal_position_msgs::msgs::MotorGoalPosition>(pos_goal_topic_);
    this->vel_goal_pub_ = this->gazebo_node_->Advertise<motor_goal_vel_msgs::msgs::MotorGoalVelocity>(vel_goal_topic_);
} 

JointMotorI::~JointMotorI() {
    gazebo::client::shutdown();
}

void  setPosition(MotorGoalPosition goal, const Ice::Current&) {
    motor_goal_position_msgs::msgs::MotorGoalPosition msg;
    msg.set_position(goal.position);
    msg.set_maxvel(goal.maxSpeed);
    msg.set_name(goal.name);

    this->pos_goal_pub_->Publish(msg);
}

void  setVelocity(MotorGoalVelocity goal, const Ice::Current&) {
    motor_goal_vel_msgs::msgs::MotorGoalVelocity msg;
    msg.set_velocity(goal.velocity);
    msg.set_maxaccel(goal.maxAcc);
    msg.set_name(goal.name);

    this->vel_goal_pub_->Publish(msg);
}
void  setZeroPos(string name, const Ice::Current&) {
    motor_goal_position_msgs::msgs::MotorGoalPosition msg;
    msg.set_position(0);
    msg.set_maxvel(10);
    msg.set_name(name);

    this->pos_goal_pub_->Publish(msg);
}

void  setSyncPosition(MotorGoalPositionList listGoals, const Ice::Current&) {

}

void  setSyncVelocity(MotorGoalVelocityList listGoals, const Ice::Current&) {

}

void  setSyncZeroPos(const Ice::Current&) {

}

MotorParams getMotorParams(string motor, const Ice::Current&) {

}

MotorState getMotorState(string motor, const Ice::Current&) {

}

MotorStateMap getMotorStateMap(MotorList mList, const Ice::Current&) {

}

void  getAllMotorState(out MotorStateMap mstateMap, const Ice::Current&) {

}

MotorParamsList getAllMotorParams(const Ice::Current&) {

}

MotorParamsList getAllMotorParams(const Ice::Current&) {

}

BusParams getBusParams(const Ice::Current&) {

}

