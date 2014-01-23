#ifndef LANDMARKSELFREF_ICE
#define LANDMARKSELFREF_ICE
/** \mainpage RoboComp Interfaces: LandmarkSelfRef.ice
 *
 * \section intro_sec Introduction
* Interface for landmarkselfrefComp  
*                                                                   
* Provide methods to get the landmarks.
*                                                                                              
*
*    PORT 10042 <br>   
*/

/** \namespace RoboCompLandmarkSelfRef
  *@brief Name space landmarkSelfRef
  */
module RoboCompLandmarkSelfRef
{
	 /** \struct Landmark
  *@brief struct landmark. 
  */
	struct Landmark
	{
		int xtop;
  		int ytop;
  		int xbottom;
  		int ybottom;
  		float length;
  		int xcenter;
  		int ycenter;
  		float angle;
  		float depth;
  		float distToCenter;
  		int xcenterCero;
  		int ycenterCero;
  		int code;
	};

	sequence<Landmark> landmarkSequence;

    /** \struct data
  *@brief struct data, two vectors of landmarks. 
  */	
	struct data
	{
		landmarkSequence left;
		landmarkSequence right;
	};	

    /** \interface LandmarkSelfRef
  *@brief interface LandmarkSelfRef
  */
	interface LandmarkSelfRef
	{
		idempotent data getData();
	};
};

#endif
