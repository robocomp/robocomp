#include <Ice/Ice.h>
#include "JointMotor.h"

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

#include <jointmotor_params.pb.h>
#include <jointmotor_state.pb.h>
#include <motor_goal_position.pb.h>
#include <motor_goal_velocity.pb.h>

#include <motor_goal_pos_list.pb.h>
#include <motor_goal_vel_list.pb.h>
#include <motor_params_list.pb.h>
#include <motor_state_list.pb.h>

typedef const boost::shared_ptr<const motor_state_list::msgs::MotorStateMap> ConstMotorStateMapPtr;
typedef const boost::shared_ptr<const motor_params_list::msgs::MotorParamsList> ConstMotorParamsListPtr;

using namespace RoboCompJointMotor;

class JointMotorI : public JointMotor
{
public: 
    JointMotorI(int argc, char **argv);
    ~JointMotorI();
	virtual void  setPosition(const MotorGoalPosition& goal, const Ice::Current&) override;
    virtual void  setVelocity(const MotorGoalVelocity& goal, const Ice::Current&) override;
    virtual void  setZeroPos(const std::string& name, const Ice::Current&) override;
    virtual void  setSyncPosition(const MotorGoalPositionList& listGoals, const Ice::Current&) override;
    virtual void  setSyncVelocity(const MotorGoalVelocityList& listGoals, const Ice::Current&) override;
    virtual void  setSyncZeroPos(const Ice::Current&) override;
    virtual MotorParams getMotorParams(const std::string& motor, const Ice::Current&) override;
    virtual MotorState getMotorState(const std::string& motor, const Ice::Current&) override;
    virtual MotorStateMap getMotorStateMap(const MotorList& mList, const Ice::Current&) override;
    virtual MotorParamsList getAllMotorParams(const Ice::Current&) override;
    virtual BusParams getBusParams(const Ice::Current&) override;

private:
    void stateCallback(ConstMotorStateMapPtr &_msg);
    void paramsCallback(ConstMotorParamsListPtr &_msg);
    
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr state_sub_;
    private: gazebo::transport::SubscriberPtr params_sub_;
    private: gazebo::transport::PublisherPtr vel_goal_pub_;
    private: gazebo::transport::PublisherPtr pos_goal_pub_;
    
    private: std::string state_topic_name_;
    private: std::string params_topic_name_;

    private: std::string vel_goal_topic_;
    private: std::string pos_goal_topic_;

    private: MotorList motor_list_;
    private: MotorParamsList motor_params_list_;
    private: MotorStateMap motor_state_map_;
}; 
