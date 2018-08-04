#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "diffdrive_cmd.pb.h"

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

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

  float linear_vel = 0;
  float angular_vel = 0;

  if (_argc < 2||_argc > 3)
  {
      std::cerr << "Invalid input." << std::endl;
      return -1;
  }

  if (_argc == 2 )
  {
      std::cerr << "Angular velocity not given." << std::endl;
      angular_vel = 0;
  }
  else
  {
      angular_vel = std::atof(_argv[2]);
  }

  linear_vel = std::atof(_argv[1]);
  
  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<diffdrive_cmd::msgs::DiffDriveCmd>("/my_robot");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  diffdrive_cmd::msgs::DiffDriveCmd msg;

  msg.set_linear_vel(linear_vel);
  msg.set_angular_vel(angular_vel);

  // Send the message
  pub->Publish(msg);

  std::cerr << "Publishing linear_vel: " << linear_vel << " angular_vel: " << angular_vel << std::endl;

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
