#ifndef GAZEBO_ROBOCOMP_DIFFDRIVE_HH
#define GAZEBO_ROBOCOMP_DIFFDRIVE_HH
 
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/gazebo.hh> 
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Angle.hh>

#include "diffdrive_state.pb.h"
#include "diffdrive.pb.h"

typedef const boost::shared_ptr<const diffdrive_state_msgs::msgs::DiffDriveState> ConstDiffDriveStatePtr;
typedef const boost::shared_ptr<const diffdrive_cmd_msgs::msgs::DiffDriveCmd> ConstDiffDriveCmdPtr;


namespace gazebo
{
  class GazeboRoboCompDiffDrive : public ModelPlugin
  {
    // Constructor
    public: GazeboRoboCompDiffDrive();

    // Destructor
    public: ~GazeboRoboCompDiffDrive();

    // Load the plugin
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Updated on each simulation iteration
    public: void OnUpdate();

    // Set the velocity of the Joint
    public: void SetVelocity(const double &_vel, const double &_ang_vel);

    // Handle incoming messages
    private: void OnMsg(ConstDiffDriveCmdPtr &_msg);

    // Topic used for communication
    private: std::string sub_topic_name_;
    private: std::string pub_topic_name_;
    // private: std::string diffdrive_state_topic_name_;
    // Pointer to the model
    private: physics::ModelPtr model_;

    // Pointer to the model
    private: sensors::RaySensorPtr parent_ray_sensor_;

    // Pointer to the joint
    private: physics::JointPtr right_joint_;

    private: physics::JointPtr left_joint_;

    // A PID controller for the joint.
    private: common::PID pid_;

    private: double wheel_separation_;

    private: double right_wheel_vel_;
    private: double left_wheel_vel_;

    private: uint32_t left_wheel_ID_;
    private: uint32_t right_wheel_ID_;

    // SDF root element
    private: sdf::ElementPtr sdf_;

    private: std::string world_name_;

    private: diffdrive_state_msgs::msgs::DiffDriveState diffdrive_state_;

    // Gazebo transport details
    private: transport::NodePtr gazebo_node_;
    private: transport::SubscriberPtr sub_;
    private: transport::PublisherPtr pub_;

    // Listen to the update event
    // The event is broadcasted every simulation iteration
    private: event::ConnectionPtr update_connection_;
  };
}
#endif
