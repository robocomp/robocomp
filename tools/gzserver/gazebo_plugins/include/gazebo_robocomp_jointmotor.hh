#ifndef GAZEBO_ROBOCOMP_JOINT_HH
#define GAZEBO_ROBOCOMP_JOINT_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <stdio.h>

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

#include <motor_goal_pos_list.pb.h>
#include <motor_goal_vel_list.pb.h>
#include <motor_params_list.pb.h>
#include <motor_state_list.pb.h>

typedef const boost::shared_ptr<const motor_goal_vel_list::msgs::MotorGoalVelocityList> ConstMotorGoalVelocityListPtr;
typedef const boost::shared_ptr<const motor_goal_pos_list::msgs::MotorGoalPositionList> ConstMotorGoalPositionListPtr;

namespace gazebo
{
    class GazeboRoboCompJointMotor : public ModelPlugin
    {
    public:
        // Constructor
        GazeboRoboCompJointMotor();

        // Destructor
        ~GazeboRoboCompJointMotor();

        // Load the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void OnVelMsg(ConstMotorGoalVelocityListPtr &_vel);
        void OnPosMsg(ConstMotorGoalPositionListPtr &_pos);
        void OnUpdate();

    private: 
        std::string pos_topic_name_;
        std::string vel_topic_name_;
        std::string state_topic_name_;
        std::string config_topic_name_;

        // World name
        std::string world_name_;

        // Pointer to the World
        physics::WorldPtr world_;

        // Pointer to the model
        physics::ModelPtr model_;

        // Pointer to the joint
        physics::JointPtr joint_;

        // Pointer to the joint controller
        physics::JointControllerPtr joint_controller_;

        std::map<std::string, physics::JointPtr> joint_map_;

        // A simple PID controller
        common::PID pid_;

        // SDF root element
        sdf::ElementPtr sdf_;

        // Gazebo transport details
        gazebo::transport::NodePtr gazebo_node_;
        gazebo::transport::SubscriberPtr vel_sub_;
        gazebo::transport::SubscriberPtr pos_sub_;
        gazebo::transport::PublisherPtr state_pub_;
        gazebo::transport::PublisherPtr config_pub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        gazebo::event::ConnectionPtr update_connection_;

        int seed;
    };
}

#endif