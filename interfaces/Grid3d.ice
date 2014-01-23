/** \mainpage RoboComp Interfaces: Grid3d.ice
 *
 * \section intro_sec Introduction
* Interface for grid3dComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef GRID3D_ICE
#define GRID3D_ICE

/** \namespace RoboCompGrid3d
  *@brief Name space Grid3d
  */

#include <Roimant.ice>
#include <Laser.ice>

module RoboCompGrid3d
{
  
  struct TCell
  {
    bool occupancy;
    float x;
    float y;
    float z;
  };
  
  struct Point3D
  {
    float x;
    float y;
    float z; 
  };
  
  sequence<TCell> TCellList;
  sequence<Point3D> Point3DList;
  
  /** \interface Grid3d
  *@brief interface Grid3d
  */ 	
  interface Grid3d
  {
  	// Adds region (TRoi) as defined in Roimant to the activecube
  	void addRoi( RoboCompRoimant::TRoi roi);
  	
  	// Adds regions as a vector of (XYZ) points 
  	void addPoints( Point3DList points);
  	
  	//Adds a RoboCompLaser::TLaserData vector of measurements
  	void addLaserData( RoboCompLaser::TLaserData laserData);

	//Update the occupancy of the cells included in cellList
	void updateCellsState(TCellList cellList);
	
	// Returns the list of occupied cells
	Point3DList getOccupiedCells();
	

  };
};

#endif
