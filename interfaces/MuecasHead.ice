/** \mainpage RoboComp Interfaces: MuecasHead.ice
 *
 * \section intro_sec Introduction
* Interface for muecasHeadComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef MUECASHEAD_ICE
#define MUECASHEAD_ICE

#include <JointMotor.ice>

/** \namespace RoboCompMuecas
  *@brief Name space Muecas
  */
module RoboCompMuecasHead
{
  /** \interface MuecasHead
  *@brief interface MuecasHead
  */ 	
  interface MuecasHead
  {
		RoboCompJointMotor::MotorParamsList getAllMotorParams();
		void  getAllMotorState(out RoboCompJointMotor::MotorStateMap mstateMap);
		void  setPosition(RoboCompJointMotor::MotorGoalPosition goal)throws RoboCompJointMotor::UnknownMotorException, RoboCompJointMotor::HardwareFailedException;
  };
};

#endif
