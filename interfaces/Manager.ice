#ifndef MANAGERDEFAULT_ICE
#define MANAGERDEFAULT_ICE

/** \mainpage RoboComp Interfaces: Logger.ice
 *
 * \section intro_sec Introduction
* Interface for managerComp.  
*                                                                   
* Component management. Displays the status of the components subscribed.
*
*/

/** \namespace RoboCompManager
  *@brief Name space manager interface.
  */

module RoboCompManager {

  sequence<string> stringVector;

	/** \interface Manager
  *@brief interface Manager
  */ 
  interface Manager
  {
    idempotent stringVector getAliases();
    
    idempotent string getEndpoint(string alias);
	///up the component specified in alias.
    void up(string alias);
	///down the componet specified in alias.
    void down(string alias);
	///reset the component specified in alias.
    void reset(string alias);
    string getConfigValue(string alias, string parameter);
  };
};

#endif
