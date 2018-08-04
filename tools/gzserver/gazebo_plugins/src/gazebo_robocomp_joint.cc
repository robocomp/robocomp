#include "gazebo_robocomp_joint.hh"

typedef const boost::shared_ptr<const motor_goal_vel::msgs::MotorGoalVelocity> ConstMotorGoalVelocityPtr;
typedef const boost::shared_ptr<const motor_goal_position::msgs::MotorGoalPosition> ConstMotorGoalPositionPtr;

namespace gazebo
{
// A plugin to control a joint of a model

// Constructor
GazeboRoboCompJoint::GazeboRoboCompJoint() {
    std::cerr << "gazebo_robocomp_joint created" << std::endl;
}

GazeboRoboCompJoint::~GazeboRoboCompJoint() {}

// The load function is called by Gazebo when the plugin is  inserted into simulation
void GazeboRoboCompJoint::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint_ = _model->GetJoints()[0];

    // Setup a P-controller, with a gain of 0.1.
    this->pid_ = common::PID(0.1, 0, 0);

    // Apply the P-controller to the joint.
    this->model_->GetJointController()->SetVelocityPID(
            this->joint_->GetScopedName(), this->pid_);

    this->model_->GetJointController()->SetPositionPID(
            this->joint_->GetScopedName(), this->pid_);

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model_->GetJointController()->SetVelocityTarget(
            this->joint_->GetScopedName(), 10.0);

    // Default to zero velocity
    double velocity = 0;

    // Check that the velocity element exists, then read the value
    if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model_->GetJointController()->SetVelocityTarget(
            this->joint_->GetScopedName(), velocity);

    // Create the node
    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pos_topic_name_ = "/position/cmd";
    this->vel_topic_name_ = "/speed/cmd";
    this->pub_topic_name_ = "/gazebo/joint/state";
    
    // Subscribe to the topic, and register a callback
    this->pos_sub_ = this->gazebo_node_->Subscribe(pos_topic_name_, &GazeboRoboCompJoint::OnPosMsg, this);
    this->vel_sub_ = this->gazebo_node_->Subscribe(vel_topic_name_, &GazeboRoboCompJoint::OnVelMsg, this);

    this->pub_ = this->gazebo_node_->Advertise<joint_motor_state::msgs::JointMotorState>(pub_topic_name_);

    this->update_connection_ = joint_->ConnectJointUpdate(boost::bind(&GazeboRoboCompJoint::OnUpdate, this));

    std::cerr << "The plugin is listening on '" << pos_topic_name_ << "'" << "topic for position goal." << std::endl;
    std::cerr << "The plugin is listening on '" << vel_topic_name_ << "'" << "topic for velocity goal." << std::endl;
    std::cerr << "The plugin is publishing joint state on '" << pub_topic_name_ << "'" << "topic." << std::endl;
}

// Set the velocity of the joint
void GazeboRoboCompJoint::SetVelocity(const double &_vel)
{
    // Set the joint's target velocity.
    this->model_->GetJointController()->SetVelocityTarget(this->joint_->GetScopedName(), _vel);
    std::cerr << "The velocity of the joint is set to: " << _vel << std::endl;
}

// Set the position of the joint
void GazeboRoboCompJoint::SetPosition(const double &_position)
{
    // Set the joint's target position.
    this->model_->GetJointController()->SetPositionTarget(this->joint_->GetScopedName(), _position);
    std::cerr << "The position of the joint is set to: " << _position << std::endl;
}

// Handle incoming message
void GazeboRoboCompJoint::OnPosMsg(ConstMotorGoalPositionPtr &_msg)
{
    this->SetPosition(_msg->position());
}

// Handle incoming message
void GazeboRoboCompJoint::OnVelMsg(ConstMotorGoalVelocityPtr &_msg)
{
    this->SetVelocity(_msg->velocity());
}

void GazeboRoboCompJoint::OnUpdate()
{
    joint_motor_state::msgs::JointMotorState msg;
    msg.set_speed(this->joint_->GetVelocity(0));
    msg.set_name(this->joint_->GetName());
    msg.set_position(0);
    msg.set_torque(this->joint_->GetForce(0));
    msg.set_timestamp("");

    pub_->Publish(msg);   
}

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboRoboCompJoint)
}