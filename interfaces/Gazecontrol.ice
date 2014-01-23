/** \mainpage RoboComp Interfaces: Gazecontrol.ice
 *
 * \section intro_sec Introduction
* Interface for gazecontrolComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef GAZECONTROL_ICE
#define GAZECONTROL_ICE

#include <CommonHead.ice>

/** \namespace RoboCompGazecontrol
  *@brief Name space Gazecontrol
  */
module RoboCompGazecontrol
{
  /** \interface Gazecontrol
  *@brief interface Gazecontrol
  */ 	
  interface Gazecontrol
  {

	  void setDominantCameraAngles(float pan, float tilt, RoboCompCommonHead::THeadState hState);
	  void setVergenceCameraAngles(float pan, float tilt, RoboCompCommonHead::THeadState hState);
	  void setSaccadic3D(float leftPan, float rightPan, float tilt);
	  void setSaccadic4D(float leftPan, float rightPan, float tilt, float neck);
  };
};

#endif
