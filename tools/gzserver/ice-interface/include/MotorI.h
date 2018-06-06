#include <Ice/Ice.h>
#include "Motors.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;
using namespace RoboCompMotors;

class MotorI : public Motors
{
public: 
    MotorI(int argc, char **argv);
    ~MotorI();
    virtual float getMotorSpeed(const Ice::Current&) override;
    virtual void setMotorSpeed(Ice::Float &angularVel, const Ice::Current&); 
private:
    void callback(ConstVector3dPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr pub_;
    private: std::string sub_topic_name_;
    private: std::string pub_topic_name_;
    private: string device_name_;
    private: float motor_speed_;
}; 
