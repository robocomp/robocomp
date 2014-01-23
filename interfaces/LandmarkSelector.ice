#ifndef LANDMARKSELECTOR_ICE
#define LANDMARKSELECTOR_ICE
/** \mainpage RoboComp Interfaces: JoyStick.ice
 *
 * \section intro_sec Introduction
* Interface for landmarkselectorComp
*                                                                   
*    
*    PORT 10020 <br>   
*    
*/
/** \namespace RoboCompLandmarkSelector
  *@brief Name space landmarkselector
  */
module RoboCompLandmarkSelector
{
    /** \interface LandmarkSelector
    *@brief interface LandmarkSelector
    */	
    interface LandmarkSelector
    {
		void startAttention();
		void stopAttention();
		void changeAttentionCat(int iCat);
		int getGlobalSelectorId();
    };
};

#endif
