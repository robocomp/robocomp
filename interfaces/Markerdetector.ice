/** \mainpage RoboComp Interfaces: Markerdetector.ice
 *
 * \section intro_sec Introduction
* Interface for markerdetectorComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef MARKERDETECTOR_ICE
#define MARKERDETECTOR_ICE

/** \namespace RoboCompMarkerdetector
  *@brief Name space Markerdetector
  */
module RoboCompMarkerdetector
{
  
  struct TVertex {
	int x;
	int y;
  };
  sequence<TVertex> TPolygon;
  
  struct TPose {
	float Tx;
	float Ty;
	float Tz;
	float alpha;
	float beta;
	float gamma;
  };

  
  /** \interface Markerdetector
  *@brief interface Markerdetector
  */ 	
  interface Markerdetector
  {
	void setSideLengthOfMarker(float sLength);
	bool getMarker(out TPolygon marker);
	bool getMarkerPose(out TPose mPose);
          
  };
  
};

#endif
