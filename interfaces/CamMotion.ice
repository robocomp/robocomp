/*
 * Dependences:
 */
#ifndef CAMMOTION_ICE
#define CAMMOTION_ICE
/** \mainpage RoboComp Interfaces: CamMotion.ice
 *
 * \section intro_sec Introduction
* Interface for CamMotionComp  
* Stereo head control.  Al angles in radians , radians/sg , etc
*
*    PORT 10005 <br>   
*/


/** \namespace RoboCompCamMotion
  *@brief Name space CamMotion
  */
module RoboCompCamMotion
{
  /**@brief exception hardware*/
  exception HardwareFailedException{ string what; };
  /**@brief exception hardware*/
  exception OutOfRangeException{ string what; };  

  /** \struct TMotorRanges
  *@brief struct motor ranges.
  * In Radians
  */	  
  struct TMotorRanges 
  {
    float min;
    float max;
    byte number;
  };
  
  /** \struct TMotorState
  *@brief struct motor State.
  * In Radians
  */
  struct TMotorState   
  {
    float pos;
    float vel;
    float power;
    /// in steps
    int p;  
    int v;
    bool isMoving;
  };
  
  /** \struct THeadRanges
  *@brief struct head ranges.
  * In Radians
  */
  struct THeadRanges   // In Radians
  {
    TMotorRanges left;
    TMotorRanges right;
    TMotorRanges tilt;
    TMotorRanges neck;
    TMotorRanges leftTilt;
    TMotorRanges rightTilt;
    int baseline;
  };
  
   /** \struct THeadState
  *@brief struct head state.
  * In Radians
  */
  struct THeadState   // In Radians
  {
    TMotorState left;
    TMotorState right;
    TMotorState tilt;
    TMotorState neck;
    TMotorState leftTilt;
    TMotorState rightTilt;
    bool isMoving;
  };
  
 /** \struct TParams
  *@brief struct head state.
  * Configuration Params  
  */
  struct TParams    
  {
    int TILTMOTOR;
    int LEFTTILTMOTOR;
    int RIGHTTILTMOTOR;
    int LEFTMOTOR;
    int RIGHTMOTOR;
    int NECKMOTOR;
    int LEFTCAMERA;
    int RIGHTCAMERA;
    int BOTHCAMERAS;
    int RIGHTZEROPOS;
    int LEFTZEROPOS;
    int TILTZEROPOS;
    int NECKZEROPOS;
    ///Communications port
    string device;  
    ///Drive for servomotor hardware
    string handler;  
    int baseline;
    bool tiltInvert;
    bool leftInvert;
    bool rightInvert;
    bool neckInvert;
  };

  /** \interface CamMotion
  *@brief interface CamMotion
  */  
  interface CamMotion
  {
    /// Send cameras to ZEROPOS and set zero speed.
    void resetHead();                   
    /// Stop head where it is now
    void stopHead();             
    /// Set PanI servo to pan rads
    void setPanLeft(float pan);         
    /// Set PanD servo to pan rads
    void setPanRight(float pan);  
    /// Set Tilt servo to tilt rads
    void setTiltLeft(float tilt);       
    /// Set Left Tilt servo to tilt rads
    void setTiltRight(float tilt);      
    /// Set both cameras to tilt rads
    void setTiltBoth(float tilt);       
    /// Set neck servo to neck radians
    void setNeck(float neck);			
    void saccadic2D(float panI, float tilt);
    void saccadic3D(float panI, float panD, float tilt);
    void getMotorRanges(byte motor, out TMotorRanges info);
    void getMotorState(byte motor, out  TMotorState state);
    void getHeadState(out THeadState state);
    void getHeadRanges(out THeadRanges ranges);

    void setRadSaccadic(float pan, float tilt, int cam);

    bool isMovingMotor(byte motor);
    bool isMovingHead();
  };
};

#endif
