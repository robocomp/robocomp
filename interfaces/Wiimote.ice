#ifndef WIIMOTE_ICE
#define WIIMOTE_ICE

/** \mainpage RoboComp Interfaces: Wiimote.ice
 *
 * \section intro_sec Introduction
* Interface for wiimoteComp.  
*                                                                   
* Interface to the remote control of the Wii console.
*
*    PORT 10043 <br>   
*/

/** \namespace RoboCompWiimote
  *@brief Name space wiimote interface.
  */

module RoboCompWiimote
{
  sequence<int> buttonVector;

  /** \interface Wiimote
  *@brief interface Wiimote
  */ 	
  interface Wiimote
  {
	/** @brief Get current accelerometer measurement */
    idempotent void getAcc(out int x, out int y, out int z); 
	/** @brief Get button state queue */
    idempotent buttonVector getButtons();                    
	/** (De)activate the led number 'pos'+1 */
    idempotent void setLed(int pos, bool active);            
	/** (De)activate rumble */
    idempotent void setRumble(bool active);                  
  };
};

#endif
