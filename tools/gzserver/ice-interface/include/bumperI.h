#include <Ice/Ice.h>
#include "bumper.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;
using namespace RoboCompBumper;

class bumperI : public Bumper 
{
public: 
    bumperI(int argc, char **argv);
    ~bumperI();
    
    virtual void enable(const Ice::Current&) override;
    virtual void disable(const Ice::Current&) override;
    virtual bool isEnabled(const Ice::Current&) override;
    virtual void reset(const Ice::Current&) override;
    virtual SensorStateMap getSensorData(const Ice::Current&) override;

private:
    void callback(ConstContactsPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: std::string topic_name_;
    private: string device_name_;
}; 
