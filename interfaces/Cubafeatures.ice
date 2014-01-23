#ifndef CUBAFEATURES_ICE
#define CUBAFEATURES_ICE

#include <Laser.ice>
/** \mainpage RoboComp Interfaces: Cubafeatures.ice
 *
 * \section intro_sec Introduction
* Interface for cubafeaturesComp  
*                                                                   
* Interface data processing and detection features for the laser.                                    
*                                                                                              
*    Notice: Y+ points upward and Z+ forward <br>                                                    
*    
*    
*    
*/


/** \namespace RoboCompCubafeatures
  *@brief Name space cubafeatures
  */

module RoboCompCubafeatures
{
  /** \struct Point
  *@brief typical struct of the coordinates of a point in space.
  */	

  sequence<float> Covariance;

  struct Point
  {
	float x;
	float y;
	float z;
	float alpha;

	Covariance R;
  };
  /** \struct Segment
  *@brief struct segment. Coordinates of the origin and end.
  */	
  struct Segment
  {
	float p1x;
	float p1y;
	float p1z;
	float p2x;
	float p2y;
	float p2z;
	float d;
	float alpha;
	float length;

	Covariance R;
  };

  /** \struct Circle
  *@brief struct circle. Center coordinates and radius
  */	
  struct Circle
  {
	float cx;
	float cy;
	float cz;
	float radius;
	Covariance R;
  };

  sequence<Point> Points;
  sequence<Segment> Segments;
  sequence<Circle> Circles;

  /** \struct Circle
  *@brief struct features. 
  */
  struct Features
  {
	Points p;
	Segments s;
	Circles c;
  };

  /** \interface Cubafeatures
  *@brief interface Cubafeatures
  */  interface Cubafeatures
  {
	/**@brief Features extracted from the readings provided by the laser */
	void computeFeatures(  RoboCompLaser::TLaserData data, out Features f);
	/**@brief CUBA Features */
	Features getFeatures();
	/**@brief CUBA Features local coordinate (robot coordinate)*/
	Features getLocalFeatures();
  };


};

#endif
