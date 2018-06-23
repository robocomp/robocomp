#include "JointMotorI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

#include <jointMotor_params.pb.h>
#include <jointMotorState.pb.h>
#include <motor_goal_position.pb.h>
#include <motor_goal_velocity.pb.h>

#include <motor_goal_pos_list.pb.h>
#include <motor_goal_vel_list.pb.h>
#include <motor_params_list.pb.h>
#include <motor_state_list.pb.h>

typedef const boost::shared_ptr<const motor_state_list::msgs::MotorStateMap> ConstMotorStateMapPtr;
typedef const boost::shared_ptr<const motor_params_list::msgs::MotorParamsList> ConstMotorParamsListPtr;

using namespace RoboCompJointMotor;
using namespace std;
using namespace gazebo;

JointMotorI::JointMotorI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);

    this->state_topic_name_ = "/gazebo/joint/state";
    this->params_topic_name_ = "/gazebo/joint/config";
    this->pos_goal_topic_ = "/speed/cmd";
    this->vel_goal_topic_ = "/position/cmd";
    
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    
    this->state_sub_ = this->gazebo_node_->Subscribe(state_topic_name_, &JointMotorI::stateCallback, this);
    this->params_sub_ = this->gazebo_node_->Subscribe(params_topic_name_, &JointMotorI::paramsCallback, this);
    this->pos_goal_pub_ = this->gazebo_node_->Advertise<motor_goal_position_msgs::msgs::MotorGoalPosition>(pos_goal_topic_);
    this->vel_goal_pub_ = this->gazebo_node_->Advertise<motor_goal_vel_msgs::msgs::MotorGoalVelocity>(vel_goal_topic_);
} 

JointMotorI::~JointMotorI() {
    gazebo::client::shutdown();
}

void  JointMotorI::setPosition(const MotorGoalPosition& goal, const Ice::Current&) {

}

void  JointMotorI::setVelocity(const MotorGoalVelocity& goal, const Ice::Current&) {

}

void  JointMotorI::setZeroPos(const string& name, const Ice::Current&) {

}

void  JointMotorI::setSyncPosition(const MotorGoalPositionList& listGoals, const Ice::Current&) {
    motor_goal_pos_list::msgs::MotorGoalPositionList msg;

    for (int i = 0; i < listGoals.size(); i++) {
        msg.mutable_motor_goal_pos_list(i)->set_position(listGoals[i].position);
        msg.mutable_motor_goal_pos_list(i)->set_maxvel(listGoals[i].maxSpeed);
        msg.mutable_motor_goal_pos_list(i)->set_name(listGoals[i].name);
    }

    pos_goal_pub_->Publish(msg);
}

void  JointMotorI::setSyncVelocity(const MotorGoalVelocityList& listGoals, const Ice::Current&) {
    motor_goal_vel_list::msgs::MotorGoalVelocityList msg;

    for (int i = 0; i < listGoals.size(); i++) {
        msg.mutable_motor_goal_vel_list(i)->set_velocity(listGoals[i].velocity);
        msg.mutable_motor_goal_vel_list(i)->set_maxaccel(listGoals[i].maxAcc);
        msg.mutable_motor_goal_vel_list(i)->set_name(listGoals[i].name);
    }

    vel_goal_pub_->Publish(msg);
}

void  JointMotorI::setSyncZeroPos(const Ice::Current&) {
    motor_goal_pos_list::msgs::MotorGoalPositionList msg;

    for (int i = 0; i < 1; i++) {
        msg.mutable_motor_goal_pos_list(i)->set_position(0);
        msg.mutable_motor_goal_pos_list(i)->set_maxvel(10);
        msg.mutable_motor_goal_pos_list(i)->set_name("");
    }

    pos_goal_pub_->Publish(msg);

}

MotorParams JointMotorI::getMotorParams(const string& motor, const Ice::Current&) {

}

MotorState JointMotorI::getMotorState(const string& motor, const Ice::Current&) {

}

MotorStateMap JointMotorI::getMotorStateMap(const MotorList& mList, const Ice::Current&) {
    return motor_state_map_;
}

MotorParamsList JointMotorI::getAllMotorParams(const Ice::Current&) {
    return motor_params_list_;
}

BusParams JointMotorI::getBusParams(const Ice::Current&) {

}

void JointMotorI::stateCallback(ConstMotorStateMapPtr &_msg){
    int joint_count_ = _msg->joint_motor_state_size();

    for (int i = 0; i < joint_count_; i++) {
        this->motor_state_map_[_msg->joint_motor_state(i).name()].p = 0;
        this->motor_state_map_[_msg->joint_motor_state(i).name()].v = 0;
        this->motor_state_map_[_msg->joint_motor_state(i).name()].temperature = 0;
        this->motor_state_map_[_msg->joint_motor_state(i).name()].isMoving = true;
        this->motor_state_map_[_msg->joint_motor_state(i).name()].pos = _msg->joint_motor_state(i).position();
        this->motor_state_map_[_msg->joint_motor_state(i).name()].vel = _msg->joint_motor_state(i).speed();
        this->motor_state_map_[_msg->joint_motor_state(i).name()].power = _msg->joint_motor_state(i).torque()*_msg->joint_motor_state(i).speed();
        this->motor_state_map_[_msg->joint_motor_state(i).name()].timeStamp = _msg->joint_motor_state(i).timestamp();
    }
}

void JointMotorI::paramsCallback(ConstMotorParamsListPtr &_msg) {
    int joint_count_ = _msg->joint_motor_params_size();
    motor_params_list_.resize(joint_count_);
    for (int i = 0; i < _msg->joint_motor_params_size(); i++) {
        motor_list_.push_back(_msg->joint_motor_params(i).name());
        motor_params_list_[i].invertedSign = 0;
		motor_params_list_[i].busId = 0;
		motor_params_list_[i].minPos = _msg->joint_motor_params(i).minpos();
		motor_params_list_[i].maxPos = _msg->joint_motor_params(i).maxpos();
		motor_params_list_[i].maxVelocity = _msg->joint_motor_params(i).maxvel();
		motor_params_list_[i].zeroPos = _msg->joint_motor_params(i).zeropos();
		motor_params_list_[i].stepsRange = _msg->joint_motor_params(i).stepsrange();
		motor_params_list_[i].maxDegrees = _msg->joint_motor_params(i).maxdegrees();
		motor_params_list_[i].offset = 0;
		motor_params_list_[i].unitsRange = 0;
		motor_params_list_[i].name = _msg->joint_motor_params(i).name();
    }
}

