#include "gazebo_robocomp_RGBD.hh"

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompRGBD)

  GazeboRoboCompRGBD::GazeboRoboCompRGBD() 
  {
    this->seed = 0;
    this->leafSize = 1./100;
    cloud = NULL;
  }
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

    this->newDepthFrameConnection = this->camera_->ConnectNewDepthFrame(
      boost::bind(&GazeboRoboCompRGBD::OnNewDepthFrame,
        this, _1, _2, _3, _4, _5));

    this->newImageFrameConnection = this->camera_->ConnectNewImageFrame(
      boost::bind(&GazeboRoboCompRGBD::OnNewImageFrame,
        this, _1, _2, _3, _4, _5));

    this->newRGBPointCloudConnection = this->camera_->ConnectNewRGBPointCloud(
      boost::bind(&DepthCameraPlugin::OnNewRGBPointCloud,
        this, _1, _2, _3, _4, _5));

    this->parentSensor->SetActive(true);

    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(this->parent_sensor_->WorldName());

    // Subscribe to the topic, and register a callback
    this->pub_ = this->gazebo_node_->Advertise<msgs::Image>(topic_name_);

  }

  // Update the controller
  void GazeboRoboCompRGBD::OnNewDepthFrame(const float *_image, unsigned int _width, unsigned int _height,
                               unsigned int _depth, const std::string &_format)
  {
    if (seed == 0)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
      cloud = cloud_;
      cloud->width = _width;
      cloud->height = _height;
      cloud->points.resize(cloud->width*cloud->height);
      imageDepth.create(_height, _width, CV_8UC3);
    }
    
    double hfov = 1.04719755;
	  double fl = ((double)_width) / (2.0 *tan(hfov/2.0));
	  double pointCloudCutoff = 0.001;

    cv::Mat image;
    image.create(_height, _width, CV_32FC1);
  
    memcpy( (float *)image.data,(float *) _image, _width*_height*4 );

    double pAngle; 
    double yAngle;

    int indicePunto = 0;
	  pcl::PointXYZRGBA point;
	  point.r      = 255;
	  point.g      = 0;
	  point.b      = 0;
  
    for(unsigned int x = 0 ; x < _width ; x++){
      for(unsigned int y = 0; y < _height; y++){
        unsigned int indice = y*_width + x;

        if (_height>1)
          yAngle = atan2( (double)x - 0.5*(double)(width-1), fl);
        else            
          yAngle = 0.0;

        if (_width>1)
          pAngle = atan2( (double)y - 0.5*(double)(height-1), fl);
        else
          pAngle = 0.0;

        float d = _image[indice];


        point.x      = d * tan(yAngle);
        point.y      = d * tan(pAngle);
        if(!imageRGB.data){
          point.r      = 255;
          point.g      = 0;
          point.b      = 0;
        }else{
          unsigned int indice = y*imageRGB.step + x*imageRGB.channels();
          point.r      = imageRGB.data[indice];
          point.g      = imageRGB.data[indice+1];
          point.b      = imageRGB.data[indice+2];
        }
        if(d > pointCloudCutoff){
          point.z    = _image[indice];
        }else{ //point in the unseeable range

          point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
          cloud->is_dense = false;
        }

        cloud->points[indicePunto++] = point;

      }
    }

	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (this->leafSize, this->leafSize, this->leafSize);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);

	sor.filter (*cloud2);

	*cloud = *cloud2;

  float * data= (float*)image.data;
  for (int i=0; i<image.rows* image.cols; i++) {
    //std::cout << i << std::endl;
    int val = (int)(data[i]*1000);
    imageDepth.data[3*i+0] = (float)val/10000*255;;
    imageDepth.data[3*i+1] = val>>8;
    imageDepth.data[3*i+2] = val&0xff;

    if(imageDepth.data[i*3]!=0)
      imageDepth.data[i*3]=255-imageDepth.data[i*3];
    imageDepth.data[i*3+1]=imageDepth.data[i*3];
    imageDepth.data[i*3+2]=imageDepth.data[i*3];

	}
  }

  // Update the controller
  void GazeboRoboCompRGBD::OnNewRGBPointCloud(const float *_pcd, unsigned int _width, unsigned int _height,
                                  unsigned int _depth, const std::string &_format)
  {

  }

  // Update the controller
  void GazeboRoboCompRGBD::OnNewImageFrame(const unsigned char *_image, unsigned int _width, unsigned int _height,
                               unsigned int _depth, const std::string &_format)
  {
    // this->imageRGB.create(_height, _width, CV_8UC3);
    if (seed == 0)
    {
      imageRGB.create(_height, _width, CV_8UC3);
    }
    seed++;

    memcpy((unsigned char *) imageRGB.data, &(_image[0]), _width*_height * 3);
    cv::imshow("Display Window - Depth Image", imageRGB);
  }
}
