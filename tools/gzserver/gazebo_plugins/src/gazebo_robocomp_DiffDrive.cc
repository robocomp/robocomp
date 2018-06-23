#include <algorithm>
#include <string>
#include <assert.h> 

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

#include "gazebo_robocomp_DiffDrive.hh"
#include "diffdrive_state.pb.h"
#include "diffdrive.pb.h"

using namespace std;

using namespace diffdrive_cmd_msgs::msgs;
using namespace diffdrive_state_msgs::msgs;

typedef const boost::shared_ptr<const diffdrive_state_msgs::msgs::DiffDriveState> ConstDiffDriveStatePtr;
typedef const boost::shared_ptr<const diffdrive_cmd_msgs::msgs::DiffDriveCmd> ConstDiffDriveCmdPtr;

namespace gazebo
{
// Register plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompDiffDrive)

GazeboRoboCompDiffDrive::GazeboRoboCompDiffDrive()
{
}

GazeboRoboCompDiffDrive::~GazeboRoboCompDiffDrive()
{
}

void GazeboRoboCompDiffDrive::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cerr << "DiffDrive Plugin Loaded!!!!" << std::endl;

  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count,pDiffDrive plugin not loaded\n";
    return;
  }

  // Store the model pointer for convenience.
  this->model_ = _model;

  this->sdf_ = _sdf;

  std::cerr << "JointCount: " << model_->GetJointCount() << std::endl;

  this->world_name_ = this->model_->GetWorld()->GetName();

  this->left_joint_ = _model->GetJoints()[0];

  if (left_joint_ == NULL)
  {
    gzthrow("ERROR: Left Joint pointer is NULL");
    return;
  }
  else 
  {
    std::cerr << "Left Joint is working." << std::endl;
  }

  this->right_joint_ = _model->GetJoints()[1];

  if (right_joint_ == NULL)
  {
    gzthrow("ERROR: Right Joint pointer is NULL");
    return;
  }
  else
  {
    std::cerr << "Right Joint is working." << std::endl;
  }

  // Setup a P-controller, with a gain of 0.1.
  this->pid_ = common::PID(0.1, 0, 0);

  // Apply the p-controller to the joint.
  this->model_->GetJointController()->SetVelocityPID(this->right_joint_->GetScopedName(), this->pid_);
  this->model_->GetJointController()->SetVelocityPID(this->left_joint_->GetScopedName(), this->pid_);

  // Default to zero velocity
  double velocity = 0;
  double angular_vel = 0;

  // Check that the velocity element exists, then read the value
  if (sdf_->HasElement("velocity"))
    velocity = sdf_->Get<double>("velocity");
  
  // Create the Node
  this->gazebo_node_ = transport::NodePtr(new transport::Node());
 
  this->gazebo_node_->Init(world_name_);

  if (!this->sdf_->HasElement("topicName"))
  {
      std::cerr <<  "DiffDrive plugin missing <topicName>, defaults to /world" << std::endl;
      this->sub_topic_name_ ="/my_robot";
  }
  else
      this->sub_topic_name_ = this->sdf_->Get<std::string>("topicName");

  this->wheel_separation_ = this->left_joint_->GetAnchor(0).Distance(this->right_joint_->GetAnchor(0));

  pub_topic_name_ = "/diffdrive/data";

  // Subscribe to the topic, and register a callback
  this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &GazeboRoboCompDiffDrive::OnMsg, this);
  this->pub_ = this->gazebo_node_->Advertise<diffdrive_state_msgs::msgs::DiffDriveState>(pub_topic_name_);
  // this->diffdrive_pub_ = this->gazebo_mode_->Advertise<gazebo::msgs::>(diffdrive_state_topic_name_);

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRoboCompDiffDrive::OnUpdate, this ) );

  std::cerr << "left joint: " << this->left_joint_->GetScopedName() << std::endl;
  std::cerr << "right joint: " << this->right_joint_->GetScopedName() << std::endl;
}
////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::SetVelocity(const double &_vel, const double &_ang_vel)
{
    // Set the joint's target velocity.

    std::cerr << "Setting linear velocity: " << _vel << " angular velocity: " << _ang_vel << std::endl;

    this->right_wheel_vel_ = _vel -  _ang_vel*wheel_separation_/2.0;
    this->left_wheel_vel_ = _vel + _ang_vel*wheel_separation_/2.0;

    this->model_->GetJointController()->SetVelocityTarget(this->right_joint_->GetScopedName(), this->right_wheel_vel_);
    this->model_->GetJointController()->SetVelocityTarget(this->left_joint_->GetScopedName(), this->left_wheel_vel_);  
}

/////////////////////////////////////////////////////////
void GazeboRoboCompDiffDrive::OnMsg(ConstDiffDriveCmdPtr &_msg)
{
  this->SetVelocity(_msg->linear_vel(), _msg->angular_vel());
  std::cerr << "Got a command for linear velocity of " << _msg->linear_vel() << " and angular velocity of " << _msg->angular_vel() << std::endl;
}

void GazeboRoboCompDiffDrive::OnUpdate() 
{
  gazebo::math::Pose base_pose = model_->GetWorldPose();
  gazebo::math::Vector3 base_lin_vel =model_->GetWorldLinearVel();
  gazebo::math::Vector3 base_lin_accln = model_->GetWorldLinearAccel();
  gazebo::math::Vector3 base_ang_vel = model_->GetWorldAngularVel();
  gazebo::math::Vector3 base_ang_accln = model_->GetWorldAngularAccel();

  ignition::math::Pose3d base_pose_ = base_pose.Ign();
  ignition::math::Vector3d base_lin_vel_ = base_lin_vel.Ign();
  ignition::math::Vector3d base_lin_accln_ = base_lin_accln.Ign();
  ignition::math::Vector3d base_ang_vel_ = base_ang_vel.Ign();
  ignition::math::Vector3d base_ang_accln_ = base_ang_accln.Ign();

  diffdrive_state_msgs::msgs::DiffDriveState msg;

  deque<gazebo::msgs::Vector3d*> states;
  gazebo::msgs::Pose* pose_;

  pose_ = msg.mutable_pose();
  states.push_back(msg.mutable_angvel());
  states.push_back(msg.mutable_angaccln());
  states.push_back(msg.mutable_linvel());
  states.push_back(msg.mutable_linaccln());
  
  deque<gazebo::msgs::Vector3d*>::iterator it;

  it = states.begin();
  (*it)->set_x(base_ang_vel_.X());
  (*it)->set_y(base_ang_vel_.Y());
  (*it)->set_z(base_ang_vel_.Z());
  
  it++;

  (*it)->set_x(base_ang_accln_.X());
  (*it)->set_y(base_ang_accln_.Y());
  (*it)->set_z(base_ang_accln_.Z());
  
  it++;

  (*it)->set_x(base_lin_vel_.X());
  (*it)->set_y(base_lin_vel_.Y());
  (*it)->set_z(base_lin_vel_.Z());
  
  it++;

  (*it)->set_x(base_lin_accln_.X());
  (*it)->set_y(base_lin_accln_.Y());
  (*it)->set_z(base_lin_accln_.Z());

  gazebo::msgs::Vector3d* pos_;
  gazebo::msgs::Quaternion* rot_;

  pos_ = pose_->mutable_position();
  rot_ = pose_->mutable_orientation();

  pos_->set_x(base_pose_.Pos().X());
  pos_->set_y(base_pose_.Pos().Y());
  pos_->set_z(base_pose_.Pos().Z());

  rot_->set_w(base_pose_.Rot().W());
  rot_->set_x(base_pose_.Rot().X());
  rot_->set_y(base_pose_.Rot().Y());
  rot_->set_z(base_pose_.Rot().Z());  

  pub_->Publish(msg);
}

}
