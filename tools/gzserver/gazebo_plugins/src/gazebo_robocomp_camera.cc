#include "gazebo_robocomp_camera.hh"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompCamera)

  GazeboRoboCompCamera::GazeboRoboCompCamera()
  {
    std::cerr << "Gazebo Camera Object created" << std::endl;
  }

  GazeboRoboCompCamera::~GazeboRoboCompCamera() {}

  void GazeboRoboCompCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    CameraPlugin::Load(_parent,_sdf);
    std::cout << "Load: " << " " <<  this->parentSensor->Camera()->Name()<< std::endl;

    this->parent_sensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    this->topic_name_ = "/gazebo_robocomp_camera/data";

    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->parent_sensor_->WorldName());
    this->pub_ = this->gazebo_node_->Advertise<gazebo::msgs::ImageStamped>(topic_name_);
  }

  void GazeboRoboCompCamera::myMemCpy(void *dest, std::string &new_image, size_t n)
  {
    // Typecast src and dest addresses to (char *)
    // char *csrc = (char *)src;
    char *cdest = (char *)dest;
    std::string::iterator it=new_image.begin();
  
    // Copy contents of src[] to dest[]
    for (int i=0; i<n; i++) {
      cdest[i] = *it;
      ++it;
    }
  }

  void GazeboRoboCompCamera::OnNewFrame(const unsigned char *_image, unsigned int _width, 
                                        unsigned int _height, unsigned int _depth, 
                                        const std::string &_format)
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), 0);
    msg.mutable_image()->set_width(_width);
    msg.mutable_image()->set_height(_height);
    msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(_format));
    msg.mutable_image()->set_step(_width*_depth);
    msg.mutable_image()->set_data(_image, msg.image().width() * _depth * msg.image().height());

    pub_->Publish(msg);                                                         
  }
}
