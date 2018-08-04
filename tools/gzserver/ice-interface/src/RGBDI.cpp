#include "RGBDI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
 
using namespace RoboCompRGBD;
using namespace std;
using namespace gazebo; 

RGBDI::RGBDI(int argc, char **argv) {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
#else
    gazebo::client::setup(argc, argv);
#endif
    this->device_name_ = "gazebo_robocomp_RGBD";
    this->sub_topic_name_ = "/gazebo_robocomp_RGBD/data";
    this->pub_topic_name_ = "/my_robot";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &RGBDI::callback, this);
    this->pub_ = this->gazebo_node_->Advertise<msgs::ImageStamped>(pub_topic_name_);
    this->seed = 0;
} 

RGBDI::~RGBDI() {
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}

TRGBDParams RGBDI::getRGBDParams(const Ice::Current&) {

}
void  RGBDI::setRegistration(Registration value, const Ice::Current&) {

}
Registration RGBDI::getRegistration(const Ice::Current&) {

}
void  RGBDI::getData(imgType& rgbMatrix, depthType& distanceMatrix, 
                    RoboCompJointMotor::MotorStateMap& hState, 
                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    storeImage(rgbMatrix);
    storeDepthImage(distanceMatrix);
}
void  RGBDI::getDepthInIR(depthType& distanceMatrix, 
                        RoboCompJointMotor::MotorStateMap& hState, 
                        RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    storeDepthImage(distanceMatrix);
}
void  RGBDI::getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, 
                    RoboCompJointMotor::MotorStateMap& hState, 
                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    storeData(color, points);
    storeDepthImage(depth);
}
void  RGBDI::getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap& hState, 
                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    storeDepthImage(depth);
}
void  RGBDI::getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap& hState, 
                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    PointSeq points;
    storeData(color, points);
}
void  RGBDI::getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, 
                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    ColorSeq color;
    storeData(color, points);
}

void RGBDI::myMemCpy(void *dest, std::string &new_image, size_t n) {
        
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

void RGBDI::callback(ConstImageStampedPtr &_msg) {
    if (seed == 0)
    {
      this->image.create(_msg->image().height(), _msg->image().width(), CV_8UC3);
      this->imageDepth.create(_msg->image().height(), _msg->image().width(), CV_8UC3);
    }
    new_image = _msg->image().data();
    myMemCpy((float *)image.data, new_image, _msg->image().width()*_msg->image().height()*4);
    seed++;
}

void RGBDI::storeDepthImage(depthType& depthArray) {

    for (int i = 0; i < image.cols*image.rows*4; i++) {
        depthArray.push_back(float(image.data[i]));
    }

}

void RGBDI::storeImage(imgType& depthArray) {

    for (int i = 0; i < image.cols*image.rows*4; i++) {
        depthArray.push_back(image.data[i]);
    }

}

void RGBDI::storeData(ColorSeq& colors, PointSeq& points) {

    float * data= (float*)image.data;
    for (int i = 0; i < image.rows* image.cols; i++) {
    
        int val = (int)(data[i]*1000);
        imageDepth.data[3*i+0] = (float)val/10000*255;;
        imageDepth.data[3*i+1] = val>>8;
        imageDepth.data[3*i+2] = val&0xff;

        if(imageDepth.data[i*3] != 0)
            imageDepth.data[i*3] = 255-imageDepth.data[i*3];
    
        imageDepth.data[i*3+1] = imageDepth.data[i*3];
        imageDepth.data[i*3+2] = imageDepth.data[i*3];
	}

    int _width = image.cols;
    int _height = image.rows;

    double hfov = 1.04719755;
    double fl = ((double)_width) / (2.0 *tan(hfov/2.0));
    double pointCloudCutoff = 0.001;
    
    double pAngle; 
    double yAngle;

    int indicePunto = 0;
    PointXYZ pointXYZ;
    ColorRGB pointRGB;

    pointRGB.red = 255;
    pointRGB.green = 0;
    pointRGB.blue = 0;
    
    for(unsigned int x = 0 ; x < _width ; x++){
      for(unsigned int y = 0; y < _height; y++){
        unsigned int indice = y*_width + x;

        if (_height>1)
          yAngle = atan2( (double)x - 0.5*(double)(_width-1), fl);
        else            
          yAngle = 0.0;

        if (_width>1)
          pAngle = atan2( (double)y - 0.5*(double)(_height-1), fl);
        else
          pAngle = 0.0;


        float d = float(image.data[indice]);

        pointXYZ.x      = d * tan(yAngle);
        pointXYZ.y      = d * tan(pAngle);

        if(!image.data){
          pointRGB.red      = 255;
          pointRGB.green      = 0;
          pointRGB.blue      = 0;
        }else{
          unsigned int indice = y*image.step + x*image.channels();
          pointRGB.red      = image.data[indice];
          pointRGB.green      = image.data[indice+1];
          pointRGB.blue      = image.data[indice+2];
        }
        if(d > pointCloudCutoff){
          pointXYZ.z    = float(image.data[indice]);
        }else{ //pointXYZ in the unseeable range

          pointXYZ.x = pointXYZ.y = pointXYZ.z = std::numeric_limits<float>::quiet_NaN ();
        }

        colors.push_back(pointRGB);
        points.push_back(pointXYZ);

      }
    }

}