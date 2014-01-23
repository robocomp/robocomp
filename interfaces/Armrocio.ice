#ifndef ARMROCIO_ICE
#define ARMROCIO_ICE

/** \mainpage RoboComp Interfaces: Armrocio.ice
 *
 * \section intro_sec Introduction
* Interface for armrocioComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/

#include <JointMotor.ice>

/** \namespace RoboCompArmrocio
  *@brief Name space Armrocio
  */
module RoboCompArmrocio
{

	enum Pose{ reposo,game1Up,game1Down,game2Up,game2Down,unknown,hola1,hola2,openni};
	
	exception PoseNotImplemented{ string what; };
	exception NoTransitionAllowed{ string what; }; 
  /** \interface Armrocio
  *@brief interface Armrocio
  */ 	
  interface Armrocio
  {
		void setPose(Pose p,string arm) throws NoTransitionAllowed, PoseNotImplemented;
		bool isMoving();
		void getNextPose(string arm,out Pose p);
		void getJointState( out RoboCompJointMotor::MotorStateMap mstateMap);
		RoboCompJointMotor::MotorParamsList getJointParams();
		void setMaxSpeed(float speed);
  };
};

#endif
