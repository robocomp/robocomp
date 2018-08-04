#include <Ice/Ice.h>
#include "IMU.h"

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

using namespace RoboCompIMU;

class IMUI : public IMU 
{
public: 
    IMUI(int argc, char **argv);
    ~IMUI();
    virtual DataImu getDataImu(const Ice::Current&) override;
    virtual Acceleration getAcceleration(const Ice::Current&) override;
    virtual Gyroscope getAngularVel(const Ice::Current&) override;
    virtual Magnetic getMagneticFields(const Ice::Current&) override;
    virtual Orientation getOrientation(const Ice::Current&) override;
    virtual void resetImu(const Ice::Current&) override;
private:
    void callback(ConstIMUPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: std::string topic_name_;
    private: std::string device_name_;
    private: DataImu imu_data_;
}; 
