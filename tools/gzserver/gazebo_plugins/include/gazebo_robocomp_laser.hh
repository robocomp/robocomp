#ifndef GAZEBO_ROBOCOMP_LASER_HH
#define GAZEBO_ROBOCOMP_LASER_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

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

        // Updated on each simulation iteration
        public: void OnNewLaserScans();
        
        public: void OnScan(ConstLaserScanStampedPtr &_msg);
        
        // Topic used for communication
        private: std::string topic_name_; 

        // World name
        private: std::string world_name_;

        // Pointer to the World
        private: physics::WorldPtr world_;

        private: sensors::RaySensorPtr parent_ray_sensor_;

        // SDF root element
        private: sdf::ElementPtr sdf_;

        // Gazebo transport details
        private: gazebo::transport::NodePtr gazebo_node_;
        private: gazebo::transport::SubscriberPtr laser_scan_sub_;
        private: gazebo::transport::PublisherPtr laser_scan_pub_;

        // Listen to the update event
        // The event is broadcasted every simulation iteration
        private: gazebo::event::ConnectionPtr updateConnection_;

        private: int seed;
    };
}
#endif
