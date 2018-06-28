#ifndef GAZEBO_ROBOCOMP_RGBD_HH
#define GAZEBO_ROBOCOMP_RGBD_HH

#include <string>

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// gazebo stuff
#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
namespace gazebo
{
  class GazeboRoboCompRGBD : public DepthCameraPlugin
  {
    public: GazeboRoboCompRGBD();
    public: ~GazeboRoboCompRGBD();
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    public: void OnNewDepthFrame(const float *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format); 

    // copied into DepthCameraPlugin
    protected: unsigned int width_, height_, depth_;
    protected: std::string camera_name_;
    protected: std::string format_;
    protected: std::string topic_name_;

    protected: sensors::SensorPtr parent_sensor_;
    protected: rendering::DepthCameraPtr camera_;

    // Gazebo transport system
    private: transport::NodePtr gazebo_node_;
    private: transport::SubscriberPtr sub_;
    private: transport::PublisherPtr pub_;

  };
}

#endif
