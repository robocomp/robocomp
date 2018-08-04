#ifndef GAZEBO_ROBOCOMP_LASER_HH
#define GAZEBO_ROBOCOMP_LASER_HH

#include <string>
#include <algorithm>
#include <assert.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <gazebo/transport/transport.hh>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>

#include "raysensor.pb.h"
#include "Laser_msgs.pb.h"

namespace gazebo
{
    class GazeboRoboCompLaser : public RayPlugin
    {
        // Constructor
        public: GazeboRoboCompLaser();

        // Destructor
        public: ~GazeboRoboCompLaser();

        // Load the plugin
        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

        public: void OnNewLaserScans();

        // World name
        private: std::string world_name_;

        private: std::string topic_name_;

        // Pointer to the World
        private: physics::WorldPtr world_;

        private: sensors::RaySensorPtr parent_ray_sensor_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        private: gazebo::transport::NodePtr gazebo_node_;
        private: gazebo::transport::PublisherPtr pub_;
    };
}
#endif
