#include <Ice/Ice.h>
#include "DifferentialRobot.h"

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

#include "diffdrive_state.pb.h" 
#include "diffdrive_cmd.pb.h"

using namespace RoboCompDifferentialRobot;

typedef const boost::shared_ptr<const diffdrive_cmd::msgs::DiffDriveCmd> DiffDriveCmdPtr;
typedef const boost::shared_ptr<const diffdrive_state::msgs::DiffDriveState> DiffDriveStatePtr;

class DifferentialRobotI : public DifferentialRobot 
{
public:  
    DifferentialRobotI(int argc, char **argv);
    ~DifferentialRobotI();
    virtual void getBaseState(RoboCompGenericBase::TBaseState&, const Ice::Current&) override;
    virtual void getBasePose(int&, int&, float&, const Ice::Current&) override;
    virtual void setSpeedBase(float, float, const Ice::Current&) override;
    virtual void stopBase(const Ice::Current&) override;
    virtual void resetOdometer(const Ice::Current&) override;
    virtual void setOdometer(const RoboCompGenericBase::TBaseState&, const Ice::Current&) override;
    virtual void setOdometerPose(int, int, float, const Ice::Current&) override;
    virtual void correctOdometer(int, int, float, const Ice::Current&) override;
private:
    void callback(DiffDriveStatePtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr pub_;
    private: std::string sub_topic_name_;
    private: std::string pub_topic_name_; 
    private: std::string device_name_;
    private: TMechParams params_;
    private: RoboCompGenericBase::TBaseState state_;
}; 
