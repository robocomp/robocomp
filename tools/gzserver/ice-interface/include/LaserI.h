#include <Ice/Ice.h>
#include "Laser.h"

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

using namespace RoboCompLaser;

#include "raysensor.pb.h"
#include "laser_data.pb.h"

typedef const boost::shared_ptr<const 
    laser_data::msgs::gazebo_robocomp_laser> ConstGazeboRoboCompLaserPtr;

class LaserI : public Laser 
{
public: 
    LaserI(int argc, char **argv);
    ~LaserI();
    virtual TLaserData getLaserData(const Ice::Current&) override;
    virtual LaserConfData getLaserConfData(const Ice::Current&) override; 
    virtual RoboCompLaser::TLaserData getLaserAndBStateData(RoboCompGenericBase::TBaseState&, 
                                                            const ::Ice::Current&) override;
private:
    void callback(ConstGazeboRoboCompLaserPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: std::string topic_name_;
    private: std::string device_name_;
    private: TLaserData LaserScanValues;
    private: LaserConfData LaserConfigData;
    private: int seed_;
}; 
