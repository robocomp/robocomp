#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include <jointMotor_params.pb.h>
#include <jointMotorState.pb.h>
#include <motor_goal_position.pb.h>
#include <motor_goal_velocity.pb.h>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(_argc, _argv);
#else
  gazebo::client::setup(_argc, _argv);
#endif

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<motor_goal_vel_msgs::msgs::MotorGoalVelocity>("/speed/cmd");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  motor_goal_vel_msgs::msgs::MotorGoalVelocity msg;

  msg.set_velocity(std::atof(_argv[1]));
  msg.set_maxaccel(10);
  msg.set_name("joint");


  // Send the message
  pub->Publish(msg);

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
