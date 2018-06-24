#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include "gazebo_robocomp_laser.hh"

#include "raysensor.pb.h"
#include "Laser_msgs.pb.h"


namespace gazebo
{

GazeboRoboCompLaser::GazeboRoboCompLaser() {}

//////////////////////////////////////////////////
GazeboRoboCompLaser::~GazeboRoboCompLaser() {}

//////////////////////////////////////////////////
void GazeboRoboCompLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    std::cerr << "Laser has been loaded!!!!" << std::endl;
    // load plugin
    RayPlugin::Load(_parent, _sdf);

    // Get the world name
    std::string world_name_ = _parent->WorldName();

    this->world_ = physics::get_world(world_name_);
    
    // save pointers
    this->sdf_ = _sdf;

    this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
        gzthrow("GazeboRoboCompLaser controller requires a Ray Sensor as its parent");
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompLaser)
}
