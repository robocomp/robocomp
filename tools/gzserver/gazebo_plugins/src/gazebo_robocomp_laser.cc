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

    this->topic_name_ = "/gazebo_robocomp_laser/data";
    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->world_name_);
    this->pub_ = this->gazebo_node_->Advertise<Laser_msgs::msgs::gazebo_robocomp_laser>(topic_name_);
}

void GazeboRoboCompLaser::OnNewLaserScans() {
    Laser_msgs::msgs::gazebo_robocomp_laser msg;
    msg.mutable_laser()->set_horizontal_max_angle(this->parent_ray_sensor_->AngleMax().Radian());
    msg.mutable_laser()->set_horizontal_min_angle(this->parent_ray_sensor_->AngleMin().Radian());
    msg.mutable_laser()->set_horizontal_resolution(this->parent_ray_sensor_->AngleResolution());
    msg.mutable_laser()->set_horizontal_samples(this->parent_ray_sensor_->RayCount());
    msg.mutable_laser()->set_vertical_samples(this->parent_ray_sensor_->VerticalRangeCount());
    msg.mutable_laser()->set_vertical_resolution(this->parent_ray_sensor_->VerticalAngleResolution());
    msg.mutable_laser()->set_vertical_min_angle(this->parent_ray_sensor_->VerticalAngleMin().Radian());
    msg.mutable_laser()->set_vertical_max_angle(this->parent_ray_sensor_->VerticalAngleMax().Radian());
    msg.mutable_laser()->set_range_max(this->parent_ray_sensor_->RangeMax());
    msg.mutable_laser()->set_range_min(this->parent_ray_sensor_->RangeMin());
    msg.mutable_laser()->set_range_resolution(this->parent_ray_sensor_->RangeResolution());

	for (int i = 0; i < this->parent_ray_sensor_->RayCount(); i++){
	    msg.set_range(i, this->parent_ray_sensor_->LaserShape()->GetRange(i));
    }

    pub_->Publish(msg);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompLaser)
}
