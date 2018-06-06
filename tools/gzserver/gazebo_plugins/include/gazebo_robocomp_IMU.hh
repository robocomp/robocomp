#ifndef GAZEBO_ROBOCOMP_IMU_HH
#define GAZEBO_ROBOCOMP_IMU_HH

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
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

  class GazeboRoboCompIMU : public SensorPlugin
  {
  public:
    // Constructor
    GazeboRoboCompIMU();

    // Destructor
    virtual ~GazeboRoboCompIMU();

    // Load the plugin
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // Updated on each simulation iteration
    void OnUpdate(const common::UpdateInfo &);

    // Add noise
    // double GaussianKernel(double mu, double sigma);

  private:
    // Topic used for communication
    std::string topic_name_;

    // World Name
    std::string world_name_;

    // Pointer to the model
    physics::ModelPtr model_;

    // Pointer to the sensor.
    sensors::ImuSensor* sensor_;
    // sensors::SensorPtr sensor_;

    // SDF root element
    sdf::ElementPtr sdf_;

    // Gazebo transport details
    transport::NodePtr gazebo_node_;

    // gazebo publisher to publish the data
    transport::PublisherPtr imu_data_publisher_;

    // gazebo Subscriber to subscribe the imu_data
    transport::SubscriberPtr imu_data_subscriber_;

    // Orientation data from the sensor.
    ignition::math::Quaterniond orientation;

    // Linear acceleration data from the sensor.
    ignition::math::Vector3d accelerometer_data;

    // Angular velocity data from the sensor.
    ignition::math::Vector3d gyroscope_data;

    // Sensor update rate.
    double update_rate;

    // Gaussian noise.
    double gaussian_noise;

    // Offset parameter, position part is unused.
    ignition::math::Pose3d offset;

    // Listen to the update event
    // The event is broadcasted every simulation iteration
    event::ConnectionPtr connection;

  };
}

#endif
