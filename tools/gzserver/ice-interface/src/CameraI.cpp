#include "CameraI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
 
using namespace RoboCompCamera;
using namespace std;
using namespace gazebo; 

CameraI::CameraI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_camera";
    this->sub_topic_name_ = "/gazebo/default/box/link/cam_sensor/image";
    this->pub_topic_name_ = "/my_robot";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->sub_ = this->gazebo_node_->Subscribe(sub_topic_name_, &CameraI::callback, this);
    this->pub_ = this->gazebo_node_->Advertise<msgs::ImageStamped>(pub_topic_name_);
    this->seed = 0;
} 

CameraI::~CameraI() {
    gazebo::client::shutdown();
}

void CameraI::getYUVImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
    cv::Mat YUVImage;
    cv::cvtColor(image, YUVImage, CV_BGR2YUV);
    roi.resize(YUVImage.cols*YUVImage.rows*3);
    cvtImageType((unsigned char *)YUVImage.data, roi, YUVImage.cols*YUVImage.rows*3);
} 

void CameraI::getYImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {

    cv::Mat YImage;
    cv::Mat YUVImage;
    cv::Mat YUVImageArray[3];

    cv::cvtColor(this->image, YUVImage, CV_BGR2YUV);
    cv::split(YUVImage, YUVImageArray);
    roi.resize(YUVImageArray[0].cols*YUVImageArray[0].rows);
    this->cvtImageType((unsigned char *)YUVImageArray[0].data, roi, YUVImageArray[0].cols*YUVImageArray[0].rows);
}

void CameraI::getYLogPolarImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
}

void CameraI::getYImageCR(int cam, int div, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {
}

void CameraI::getRGBPackedImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {

    for (int i = 0; i < image_.size(); i++) {
        roi[i] = image_[i];
    }
}

void CameraI::getYRGBImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) {

}

TCamParams CameraI::getCamParams(const Ice::Current&) {

}

void CameraI::setInnerImage(const imgType& roi, const Ice::Current&) {

}

void CameraI::storeImage(imgType &image_, std::string &new_image, size_t n) {

    std::string::iterator it=new_image.begin();

    for (int i = 0; i < n; i++) {
        image_[i] = *it;
        ++it;
    }
}

void CameraI::cvtImageType(void *src, imgType &image_, size_t n) {
    
    char *csrc = (char *)src;
    for (int i = 0; i < n; i++) {
        image_[i] = csrc[i];
    }
}

void CameraI::myMemCpy(void *dest, std::string &new_image, size_t n)
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

void CameraI::callback(ConstImageStampedPtr &_msg) {
    if (seed == 0)
    {
      this->image.create(_msg->image().height(), _msg->image().width(), CV_8UC3);
      this->image_.resize(_msg->image().height()*_msg->image().width()*3);
    }
    std::string new_image;
    new_image = _msg->image().data();
    myMemCpy((unsigned char *)image.data, new_image, _msg->image().width()*_msg->image().height()*3);
    storeImage(image_, new_image, _msg->image().width()*_msg->image().height()*3);
    cv::imshow("window", image);
    seed++;
}