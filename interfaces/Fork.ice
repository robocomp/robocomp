/** \mainpage RoboComp Interfaces: Fork.ice
 *
 * \section intro_sec Introduction
* Interface for forkComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef FORK_ICE
#define FORK_ICE

/** \namespace RoboCompFork
  *@brief Name space Fork
  */
module RoboCompFork
{
  /** \interface Fork
  *@brief interface Fork
  */ 	
  interface Fork
  {
		void goDown();
		void goUp();
		void setPosition(int position);
		void setSpeed(int speed);
		int getPosition();
		void stop();
		void setUpPosition();
  };
};

#endif
