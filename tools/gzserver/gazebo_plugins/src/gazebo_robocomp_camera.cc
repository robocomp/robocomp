#include "gazebo_robocomp_camera.hh"
#include <gazebo/rendering/Camera.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image.pb.h>

using namespace cv;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompCamera)

  GazeboRoboCompCamera::GazeboRoboCompCamera()
  {
    this->seed = 0;
    this->saveCount = 0;
    this->seed_ = 0;
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

    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->parent_sensor_->WorldName());
    this->sub_ = this->gazebo_node_->Subscribe("/gazebo/default/box/link/cam_sensor/image", &GazeboRoboCompCamera::OnMsg, this);
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

  void GazeboRoboCompCamera::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format)
  {
    if (seed_ == 0)
    {
      image_.create(_height, _width, CV_8UC3);
      std::cerr << "Image created!!!" << std::endl;
    }
    memcpy((unsigned char *) image_.data, &(_image[0]), _width*_height * 3);

  }

  void GazeboRoboCompCamera::OnMsg(ConstImageStampedPtr &_msg) {
    if (seed == 0)
    {
      this->image.create(_msg->image().height(), _msg->image().width(), CV_8UC3);
    }
    new_image = _msg->image().data();
    myMemCpy((unsigned char *)image.data, new_image, _msg->image().width()*_msg->image().height()*3);
    cv::imshow("window", image);
  }
}
