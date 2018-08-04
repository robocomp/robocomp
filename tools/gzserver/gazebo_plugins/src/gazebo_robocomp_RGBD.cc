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
    
    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->parent_sensor_->WorldName());

    this->topic_name_ = "/gazebo_robocomp_RGBD/data";

    // Subscribe to the topic, and register a callback
    this->pub_ = this->gazebo_node_->Advertise<msgs::ImageStamped>(topic_name_);

  }

  void GazeboRoboCompRGBD::OnNewDepthFrame(const float *_image, unsigned int _width, 
                                          unsigned int _height, unsigned int _depth, 
                                          const std::string &_format) 
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), 0);
    msg.mutable_image()->set_width(_width);
    msg.mutable_image()->set_height(_height);
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);

    unsigned int depthSamples = msg.image().width() * msg.image().height();
    float f;
    unsigned int depthBufferSize = depthSamples * sizeof(f);

    msg.mutable_image()->set_data(_image, depthBufferSize);
    this->pub_->Publish(msg);
  }
}
