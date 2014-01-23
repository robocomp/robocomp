/** \mainpage RoboComp Interfaces: Mouth.ice
 *
 * \section intro_sec Introduction
* Interface for mouthComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef MOUTH_ICE
#define MOUTH_ICE

/** \namespace RoboCompMouth
  *@brief Name space Mouth
  */
module RoboCompMouth
{

  sequence<float> Dialog;  

  /** \interface Mouth
  *@brief interface Mouth
  */ 	
  interface Mouth
  {
	void moveMouth(Dialog d);
	void selectMovement(int type);//0 => entropy, 1 => random, 2 =>

  };
};

#endif
