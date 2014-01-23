/** \mainpage RoboComp Interfaces: Steeringwheels.ice
 *
 * \section intro_sec Introduction
* Interface for steeringwheelsComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef STEERINGWHEELS_ICE
#define STEERINGWHEELS_ICE

/** \namespace RoboCompSteeringwheels
  *@brief Name space Steeringwheels
  */
module RoboCompSteeringwheels
{
  /** \struct Motion  
  *@brief struct Motion: Set the motion of mobile robot  
  */ 	
  /*struct Motion    
  {
    float vx;         
    float vy;
    int icr; //coordenate of ICR over y frame       
  };
  
  sequence<Motion> MotionList;*/
  
  /** \interface Steeringwheels
  *@brief interface Steeringwheels
  */ 	
  interface Steeringwheels
  {
	void setWheelTrack(int wheelTrack);
	void setMotion (float vx, float vy, int icr);
  };
};

#endif
