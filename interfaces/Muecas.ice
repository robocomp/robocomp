/** \mainpage RoboComp Interfaces: Muecas.ice
 *
 * \section intro_sec Introduction
* Interface for muecasComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef MUECAS_ICE
#define MUECAS_ICE

/** \namespace RoboCompMuecas
  *@brief Name space Muecas
  */
module RoboCompMuecas
{
  /** \interface Muecas
  *@brief interface Muecas
  */ 	
  interface Muecas
  {
	void setMouth(float pos);
  };
};

#endif
