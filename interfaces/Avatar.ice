/** \mainpage RoboComp Interfaces: Avatar.ice
 *
 * \section intro_sec Introduction
* Interface for avatarComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef AVATAR_ICE
#define AVATAR_ICE

/** \namespace RoboCompAvatar
  *@brief Name space Avatar
  */
module RoboCompAvatar
{
	
  /** \interface Avatar
  *@brief interface Avatar
  */ 	
  interface Avatar
  {
	void setMouth(float pos);
  };
};

#endif
