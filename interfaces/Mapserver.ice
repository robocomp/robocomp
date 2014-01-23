/** \mainpage RoboComp Interfaces: Mapserver.ice
 *
 * \section intro_sec Introduction
* Interface for mapserverComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef MAPSERVER_ICE
#define MAPSERVER_ICE

/** \namespace RoboCompMapserver
  *@brief Name space Mapserver
  */
module RoboCompMapserver
{
  
  struct TCell
  {
    float occupacy;
    float x;
    float y;
    float z;
  };
  
  
  sequence<TCell> TCellList;

  
  /** \interface Mapserver
  *@brief interface Mapserver
  */ 	
  interface Mapserver
  {
	   TCellList getMap();
  };
};

#endif
