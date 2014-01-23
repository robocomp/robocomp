/** \mainpage RoboComp Interfaces: Smar.ice
 *
 * \section intro_sec Introduction
* Interface for smarComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef SMAR_ICE
#define SMAR_ICE

/** \namespace RoboCompSmar
  *@brief Name space Smar
  */
module RoboCompSmar
{

	 exception HardwareFailedException { string what; };

  /** \struct TMechParams
  *@brief Struct mechanical params
  */
  struct TMechParams
  {
	/// Wheel radius in mm
    int wheelRadius;
	/// Axis length in mm
    int axisLength;
	/// Encoders steps
    int encoderSteps;
	/// Gear ratio
    int gearRatio;
    float temp;
    string device;
    string handler;
	/// Maximum advacen velocity
    float maxVelAdv;
	// Maximum rotation velocity
    float maxVelRot;
  };
  
   struct TBaseState
  {
    ///sideways displacement in mm
    float x; 
    ///from displacement in mm  
    float z;    
    ///angle rotated since last reset in rads
    float alpha;
    ///current advance speed
    float advV; 
    ///current rotation speed
    float rotV; 
    ///Incremental measures
    float adv;  
    float rot;
    bool isMoving;
  };
  
  


  /** \interface Smar
  *@brief interface Smar
  */ 	
  interface Smar
  {
  
	   ///Get Base local state
	  void getBaseState(out TBaseState state) throws HardwareFailedException;        
	  
	  ///provided for convenience
	  void getBasePose(out int x, out int z, out float alpha)throws  HardwareFailedException;         

	  /// Set base advance and rotation speed
	  void setSpeedBase(float adv, float rot)throws  HardwareFailedException;         

	  /// Stop the base
	  void stopBase()throws  HardwareFailedException;         

	  ///Reset Odometer
	  void resetOdometer()throws  HardwareFailedException;         
	  
	  ///Set Odometer
	  void setOdometer(TBaseState state)throws  HardwareFailedException;         

	  ///provided for convenience
	  void setOdometerPose(int x, int z, float alpha)throws  HardwareFailedException;   
  

  };
};

#endif
