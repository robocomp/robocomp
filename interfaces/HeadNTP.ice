/*
 * Dependences:
 */
#ifndef HEADNTP_ICE
#define HEADNTP_ICE
/** \mainpage RoboComp Interfaces: HeadNTP.ice
 *
 * \section intro_sec Introduction
* Interface for HeadNTPComp  
* Stereo head control.  All angles in radians , radians/sg , etc
*
*    PORT 10005 <br>   
*/

#include <JointMotor.ice>
#include <CommonHead.ice>

/** \namespace RoboCompHeadNTP
  *@brief Name space HeadNTP
  */
module RoboCompHeadNTP
{
    

  /** \interface HeadNTP
  *@brief interface HeadNTP
  */  
  interface HeadNTP
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
    /// Set neck servo to neck radians
    void setNeck(float neck) throws RoboCompJointMotor::OutOfRangeException;
    void saccadic2DLeft(float leftPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void saccadic2DRight(float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void saccadic3D(float leftPan, float rightPan, float tilt) throws RoboCompJointMotor::OutOfRangeException; 
    void saccadic4D(float leftPan, float rightPan, float tilt, float neck) throws RoboCompJointMotor::OutOfRangeException; 
    void setNMotorsPosition(RoboCompJointMotor::MotorGoalPositionList listGoals) throws RoboCompJointMotor::OutOfRangeException; 
    void getHeadState(out RoboCompCommonHead::THeadState hState);
    RoboCompCommonHead::THeadParams getHeadParams();
    bool isMovingHead();
  };
};

#endif
