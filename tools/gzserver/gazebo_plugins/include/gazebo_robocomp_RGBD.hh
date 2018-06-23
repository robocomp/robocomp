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

// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/project_inliers.h>

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

    // Update the controller
    public: void OnNewDepthFrame(const float *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    // Update the controller
    public: void OnNewRGBPointCloud(const float *_pcd,
                    unsigned int _width, unsigned int _height,
                    unsigned int _depth, const std::string &_format);

    // Update the controller
    public: void OnNewImageFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    public: void depth2rgb(cv::Mat image);

    // Gazebo transport system
    private: transport::NodePtr gazebo_node_;
    private: transport::SubscriberPtr sub_;
    private: transport::PublisherPtr pub_;

    // copied into DepthCameraPlugin
    protected: unsigned int width_, height_, depth_;
    protected: std::string camera_name_;
    protected: std::string format_;
    protected: std::string topic_name_;

    protected: sensors::SensorPtr parent_sensor_;
    protected: rendering::DepthCameraPtr camera_;

    private: event::ConnectionPtr newDepthFrameConnection;
    private: event::ConnectionPtr newRGBPointCloudConnection;
    private: event::ConnectionPtr newImageFrameConnection;

    protected: float leafSize;

    private: cv::Mat imageRGB;
    private: cv::Mat imageDepth;
    // private: pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    private: int seed;
    private: int seed_;

  };
}

#endif
