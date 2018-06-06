#include "gazebo_robocomp_jointmotor.hh"

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <algorithm>
#include <string>
#include <assert.h>

#include <motor_goal_pos_list.pb.h>
#include <motor_goal_vel_list.pb.h>
#include <motor_params_list.pb.h>
#include <motor_state_list.pb.h>

typedef const boost::shared_ptr<const motor_goal_vel_list::msgs::MotorGoalVelocityList> ConstMotorGoalVelocityListPtr;
typedef const boost::shared_ptr<const motor_goal_pos_list::msgs::MotorGoalPositionList> ConstMotorGoalPositionListPtr;

namespace gazebo
{
// A plugin to control a joint of a model

// Constructor
GazeboRoboCompJointMotor::GazeboRoboCompJointMotor() {
    std::cerr << "gazebo_robocomp_joint created" << std::endl;
}

GazeboRoboCompJointMotor::~GazeboRoboCompJointMotor() {}

// The load function is called by Gazebo when the plugin is  inserted into simulation
void GazeboRoboCompJointMotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Just output a message for now
    std::cerr << "\nThe velodyne plugin is attach to model[" << _model->GetName() << "]\n";

    // Safety check
    if (_model->GetJointCount() == 0)
    {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
    }

    // Store the model pointer for convenience.
    this->model_ = _model;

    this->world_name_ = _model->GetWorld()->GetName();

    this->joint_controller_ = this->model_->GetJointController();

    joint_map_ = joint_controller_->GetJoints();

    // Setup a P-controller, with a gain of 0.1.
    this->pid_ = common::PID(0.1, 0, 0);

    for (int i = 0; i < model_->GetJointCount(); i++)
    {
        this->joint_controller_->SetVelocityPID(_model->GetJoints()[i]->GetScopedName(), this->pid_);
    }

    for (int i = 0; i < model_->GetJointCount(); i++)
    {
        this->joint_controller_->SetPositionPID(_model->GetJoints()[i]->GetScopedName(), this->pid_);
    }

    // Create the node
    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pos_topic_name_ = "/position/cmd";
    this->vel_topic_name_ = "/speed/cmd";
    this->state_topic_name_ = "/gazebo/joint/state";
    this->config_topic_name_ = "/gazebo/joint/config";
    
    // Subscribe to the topic, and register a callback
    this->pos_sub_ = this->gazebo_node_->Subscribe(pos_topic_name_, &GazeboRoboCompJointMotor::OnPosMsg, this);
    this->vel_sub_ = this->gazebo_node_->Subscribe(vel_topic_name_, &GazeboRoboCompJointMotor::OnVelMsg, this);

    this->state_pub_ = this->gazebo_node_->Advertise<motor_state_list::msgs::MotorStateMap>(state_topic_name_);
    this->config_pub_ = this->gazebo_node_->Advertise<motor_params_list::msgs::MotorParamsList>(config_topic_name_);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                std::bind(&GazeboRoboCompJointMotor::OnUpdate, this));

    std::cerr << "The plugin is listening on '" << pos_topic_name_ << "'" << "topic for position goal." << std::endl;
    std::cerr << "The plugin is listening on '" << vel_topic_name_ << "'" << "topic for velocity goal." << std::endl;
    std::cerr << "The plugin is publishing joint state on '" << state_topic_name_ << "'" << "topic." << std::endl;
    std::cerr << "The plugin is publishing joint config on '" << config_topic_name_ << "'" << "topic." << std::endl;
}

// Handle incoming message
void GazeboRoboCompJointMotor::OnPosMsg(ConstMotorGoalPositionListPtr &_msg)
{
    for (int i = 0; i < model_->GetJointCount(); i++) {
        this->joint_controller_->SetPositionTarget(_msg->motor_goal_pos_list(i).name(), _msg->motor_goal_pos_list(i).position());
    }
}

// Handle incoming message
void GazeboRoboCompJointMotor::OnVelMsg(ConstMotorGoalVelocityListPtr &_msg)
{
    for (int i = 0; i < model_->GetJointCount(); i++) {
        this->joint_controller_->SetVelocityTarget(_msg->motor_goal_vel_list(i).name(), _msg->motor_goal_vel_list(i).velocity());
    }
}

void GazeboRoboCompJointMotor::OnUpdate()
{
    motor_state_list::msgs::MotorStateMap state_msg;
    motor_params_list::msgs::MotorParamsList params_msg;

    for (int i = 0; i < model_->GetJointCount(); i++) {

    }
}

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompJointMotor)
}