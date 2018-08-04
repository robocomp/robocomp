#ifndef GAZEBO_ROBOCOMP_CAMERA_HH
#define GAZEBO_ROBOCOMP_CAMERA_HH

#include <string>
#include <iostream>

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// sdf stuff
#include <sdf/Param.hh>
#include <sdf/sdf.hh>

// Gazebo
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/rendering/Camera.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

namespace gazebo
{
  class GazeboRoboCompCamera: public CameraPlugin
  {
      // Constructor
      public: GazeboRoboCompCamera();

      // Deconstructor
      public: ~GazeboRoboCompCamera();

      // Load the plugin
      public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

      // Update the controller
      public: void OnNewFrame(const unsigned char *_image, unsigned int _width, 
                              unsigned int _height, unsigned int _depth, const std::string &_format);
      
      public: void myMemCpy(void *dest, std::string &new_image, size_t n);

      // copied from CameraPlugin
      private: unsigned int width_, height_, depth_;
      private: std::string camera_name_;
      private: std::string format_;

      private: sensors::SensorPtr parent_sensor_;
      private: rendering::CameraPtr camera_;

      // Pointer to the world
      private: physics::WorldPtr world_;

      private: event::ConnectionPtr new_frame_connection_;

      private: common::Time sensor_update_time_;

      private: sdf::ElementPtr sdf_;

      // Gazebo transport details
      private: gazebo::transport::NodePtr gazebo_node_;
      private: gazebo::transport::SubscriberPtr sub_;
      private: gazebo::transport::PublisherPtr pub_;

      private: std::string new_image;

      // Topic used for communication
      private: std::string topic_name_;
  };
}
#endif
