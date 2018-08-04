#include <Ice/Ice.h>
#include "Camera.h"

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace RoboCompCamera;

class CameraI : public Camera 
{
public:  
    CameraI(int argc, char **argv);
    ~CameraI();
    virtual void getYUVImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                            RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void getYImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                        RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void getYLogPolarImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                                RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void getYImageCR(int cam, int div, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                            RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void getRGBPackedImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                                    RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void getYRGBImage(int cam, imgType& roi, RoboCompCommonHead::THeadState& hState, 
                            RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual TCamParams getCamParams(const Ice::Current&) override;
    virtual void setInnerImage(const imgType& roi, const Ice::Current&) override;
private:
    void callback(ConstImageStampedPtr &_msg);
    void storeImage(imgType &image_, std::string &new_image, size_t n);
    void cvtImageType(void *src, imgType &image_, size_t n);
    void myMemCpy(void *dest, std::string &new_image, size_t n);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr pub_;
    private: std::string sub_topic_name_;
    private: std::string pub_topic_name_; 
    private: string device_name_;
    private: imgType image_;
    private: cv::Mat image;
    private: TCamParams params_;
    private: int seed;
}; 
