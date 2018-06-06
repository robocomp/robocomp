#ifndef GAZEBO_ROBOCOMP_BUMPER_HH
#define GAZEBO_ROBOCOMP_BUMPER_HH

#include <map>
#include <string>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
 
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/ContactSensor.hh>

#include <gazebo/common/Exception.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/sensors/SensorTypes.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GazeboRoboCompBumper : public SensorPlugin
  {
    // Constructor
    public: GazeboRoboCompBumper();

    // Destructor 
    public: ~GazeboRoboCompBumper();

    // Load the plugin
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // Update the controller
    private: void OnContact();

    // pointer to ros node
    private: transport::NodePtr gazebo_node_;
    private: transport::PublisherPtr pub_;

    private: sensors::ContactSensorPtr parent_sensor_;

    /// \brief set topic name of broadcast
    private: std::string bumper_topic_name_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
  };

}

#endif
