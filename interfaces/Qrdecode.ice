/** \mainpage RoboComp Interfaces: Qrdecode.ice
 *
 * \section intro_sec Introduction
* Interface for qrdecodeComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef QRDECODE_ICE
#define QRDECODE_ICE

/** \namespace RoboCompQrdecode
  *@brief Name space Qrdecode
  */
module RoboCompQrdecode
{


  sequence<byte> imgType;

  /** \interface Qrdecode
  *@brief interface Qrdecode
  */ 	
  interface Qrdecode
  {

	string decode(imgType img,int width,int height);


  };
};

#endif
