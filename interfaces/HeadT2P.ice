/*
 * Dependences:
 */
#ifndef HEADT2P_ICE
#define HEADT2P_ICE
/** \mainpage RoboComp Interfaces: HeadT2P.ice
*
* \section intro_sec Introduction
* Interface for HeadT2PComp  
* Stereo head control.  All angles in radians , radians/sg , etc
*
*    PORT 10005 <br>   
*/

#include <JointMotor.ice>
#include <CommonHead.ice>

/** \namespace RoboCompHeadT2P
  *@brief Name space HeadT2P
  */
module RoboCompHeadT2P
{

  
  dictionary<string,RoboCompJointMotor::MotorParams> dmotorParams;
  struct THeadParams
  {
    dmotorParams motorsParams;
    string model;
    string tiltMotorName;
    string leftPanMotorName;
    string rightPanMotorName;
  };

  
  /** \interface HeadT2P
  *@brief interface HeadT2P
  */
  
  
  interface HeadT2P
  {
    /// Send cameras to ZEROPOS and set zero speed.
    void resetHead();                   
    /// Stop head where it is now
    void stopHead();             
    /// Set PanI servo to pan rads
    void setPanLeft(float pan) throws RoboCompJointMotor::OutOfRangeException;         
    /// Set PanD servo to pan rads
    void setPanRight(float pan) throws RoboCompJointMotor::OutOfRangeException;  
    /// Set Tilt servo to tilt rads
    void setTilt(float tilt) throws RoboCompJointMotor::OutOfRangeException; 		
    void saccadic2DLeft(float leftPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void saccadic2DRight(float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void saccadic3D(float leftPan, float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void setNMotorsPosition(RoboCompJointMotor::MotorGoalPositionList listGoals) throws RoboCompJointMotor::OutOfRangeException; 
    void getHeadState(out RoboCompCommonHead::THeadState hState);
    RoboCompCommonHead::THeadParams getHeadParams();
    bool isMovingHead();
  };
};

#endif
