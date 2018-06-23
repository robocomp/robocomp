#include <Ice/Ice.h>
#include "Motors.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

#include <jointMotor_params.pb.h>
#include <jointMotorState.pb.h>
#include <motor_goal_position.pb.h>
#include <motor_goal_velocity.pb.h>

typedef const boost::shared_ptr<const jointMotorState_msgs::msgs::JointMotorState> ConstJointMotorStatePtr;

using namespace std;
using namespace RoboCompMotors;

class MotorI : public Motors
{
public: 
    MotorI(int argc, char **argv);
    ~MotorI();
    virtual void setPosition(const MotorGoalPosition& goal, const Ice::Current&) override;
    virtual void setVelocity(const MotorGoalVelocity& goal, const Ice::Current&) override;
    virtual MotorState getState(const Ice::Current&) override;
    virtual void setZeroPos(const Ice::Current&) override;

private:
    void callback(ConstJointMotorStatePtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr pos_pub_;
    private: gazebo::transport::PublisherPtr vel_pub_;
    
    private: std::string sub_topic_name_;
    private: std::string pos_topic_name_;
    private: std::string vel_topic_name_;
    private: MotorState motor_state_;
}; 
