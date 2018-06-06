#include <Ice/Ice.h>
#include "RGBD.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace RoboCompRGBD;

class RGBDI : public RGBD 
{
public:  
    RGBDI(int argc, char **argv);
    ~RGBDI();
    virtual TRGBDParams getRGBDParams(const Ice::Current&) override;
    virtual void  setRegistration(Registration value, const Ice::Current&) override;
    virtual Registration getRegistration(const Ice::Current&) override;
    virtual void  getData(imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void  getDepthInIR(depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
    virtual void  getImage(ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
	virtual	void  getDepth(DepthSeq& depth, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
	virtual	void  getRGB(ColorSeq& color, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;
	virtual	void  getXYZ(PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current&) override;

private:
    void callback(ConstImageStampedPtr &_msg);
    void storeData(ColorSeq& colors, PointSeq& points);
    void storeImage(imgType&);
    void storeDepthImage(depthType&);
    void myMemCpy(void *dest, std::string &new_image, size_t n);

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr pub_;

    private: std::string sub_topic_name_;
    private: std::string pub_topic_name_; 
    private: string device_name_;
    private: std::string new_image;

    private: imgType image_data_;
    private: depthType depthImage_data_;

    private: ColorSeq image_;
    private: PointSeq depthImage_;

    private: cv::Mat image;
    private: cv::Mat imageDepth;

    private: CameraParameters camParams;
    private: TRGBDParams rgbdParams;
    private: int seed;
}; 
