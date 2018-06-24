#include "gazebo_robocomp_RGBD.hh"

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompRGBD)

  GazeboRoboCompRGBD::GazeboRoboCompRGBD() {}
  GazeboRoboCompRGBD::~GazeboRoboCompRGBD() {} 

  void GazeboRoboCompRGBD::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    DepthCameraPlugin::Load(_parent, _sdf);

    std::cerr << "Depth Camera plugin loaded." << std::endl;
    this->parent_sensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->depthCamera;

    if (!this->parent_sensor_)
    {
      gzerr << "DepthCameraPlugin is not attached to a depthCamera sensor\n";
      return;
    }

    this->parentSensor->SetActive(true);
  }
}
