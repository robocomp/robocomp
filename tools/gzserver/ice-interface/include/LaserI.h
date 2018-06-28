#include <Ice/Ice.h>
#include "Laser.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;
using namespace RoboCompLaser;

#include "raysensor.pb.h"
#include "Laser_msgs.pb.h"

typedef const boost::shared_ptr<const Laser_msgs::msgs::gazebo_robocomp_laser> ConstGazeboRoboCompLaserPtr;

class LaserI : public Laser 
{
public: 
    LaserI(int argc, char **argv);
    ~LaserI();
    virtual TLaserData getLaserData(const Ice::Current&) override;
    virtual LaserConfData getLaserConfData(const Ice::Current&) override; 
    virtual RoboCompLaser::TLaserData getLaserAndBStateData(RoboCompGenericBase::TBaseState&, const ::Ice::Current&) override;
private:
    void callback(ConstGazeboRoboCompLaserPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: std::string topic_name_;
    private: string device_name_;
    private: TLaserData LaserScanValues;
    private: LaserConfData LaserConfigData;
    private: int seed_;
}; 
