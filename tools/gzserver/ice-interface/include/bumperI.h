#include <Ice/Ice.h>
#include "bumper.h"

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
