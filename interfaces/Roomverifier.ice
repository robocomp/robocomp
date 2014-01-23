/** \mainpage RoboComp Interfaces: Roomverifier.ice
 *
 * \section intro_sec Introduction
* Interface for roomverifierComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef ROOMVERIFIER_ICE
#define ROOMVERIFIER_ICE

/** \namespace RoboCompRoomverifier
  *@brief Name space Roomverifier
  */
module RoboCompRoomverifier
{
  /** \interface Roomverifier
  *@brief interface Roomverifier
  */ 	
  interface Roomverifier
  {
	  void verfiyNewRoom(float xc, float yc, float alpha, float w, float h);
	  void updateRoomParameters(float xc, float yc, float alpha, float w, float h);
	  bool isRoomVerified();
	  void stopVerifying();
	  bool getCurrentTarget(out float targetX, out float targetZ);

  };
};

#endif
