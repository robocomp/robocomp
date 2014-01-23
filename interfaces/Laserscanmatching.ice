/** \mainpage RoboComp Interfaces: Laserscanmatching.ice
 *
 * \section intro_sec Introduction
* Interface for laserscanmatchingComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef LASERSCANMATCHING_ICE
#define LASERSCANMATCHING_ICE
#include <Laser.ice>
/** \namespace RoboCompLaserscanmatching
  *@brief Name space Laserscanmatching
  */
module RoboCompLaserscanmatching
{
  
  sequence<float> Covariance;
  
  /**@brief struct Point */
    struct Motion
  {
	float x;
	float y;
	float z;
	float alpha;

	Covariance R;
  };
  
  /** \interface Laserscanmatching
  *@brief interface Laserscanmatching
  */ 	
  interface Laserscanmatching
  {
	/**@brief displacement estimate from the scan matching algorithm */
	Motion getDisplacement();
	
	/**@brief displacement estimate from the readings provided by the laser */
	Motion computeDisplacement(RoboCompLaser::TLaserData datacurrent, RoboCompLaser::TLaserData dataref);
	
  };
};

#endif
