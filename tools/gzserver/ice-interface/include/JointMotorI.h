#include <Ice/Ice.h>
#include "JointMotor.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;
using namespace RoboCompJointMotor;

class JointMotorI : public JointMotor
{
public: 
    JointMotorI(int argc, char **argv);
    ~JointMotorI();
	virtual void  setPosition(MotorGoalPosition goal, const Ice::Current&) override;
    virtual void  setVelocity(MotorGoalVelocity goal, const Ice::Current&) override;
    virtual void  setZeroPos(string name, const Ice::Current&) override;
    virtual void  setSyncPosition(MotorGoalPositionList listGoals, const Ice::Current&) override;
    virtual void  setSyncVelocity(MotorGoalVelocityList listGoals, const Ice::Current&) override;
    virtual void  setSyncZeroPos(const Ice::Current&) override;
    virtual MotorParams getMotorParams(string motor, const Ice::Current&) override;
    virtual MotorState getMotorState(string motor, const Ice::Current&) override;
    virtual MotorStateMap getMotorStateMap(MotorList mList, const Ice::Current&) override;
    virtual void  getAllMotorState(out MotorStateMap mstateMap, const Ice::Current&) override;
    virtual MotorParamsList getAllMotorParams(const Ice::Current&) override;
    virtual MotorParamsList getAllMotorParams(const Ice::Current&) override;
    virtual BusParams getBusParams(const Ice::Current&) override;

private:
    void callback(ConstVector3dPtr &_msg);
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: gazebo::transport::PublisherPtr vel_goal_pub_;
    private: gazebo::transport::PublisherPtr pos_goal_pub_;
    
    private: std::string sub_topic_name_;
    private: std::string vel_goal_topic_;
    private: std::string pos_goal_topic_
    private: string device_name_;
    private: float motor_speed_;
}; 
