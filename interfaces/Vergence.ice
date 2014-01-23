#ifndef VERGENCE_ICE
#define VERGENCE_ICE

#include <Vision.ice>

/** \mainpage RoboComp Interfaces: Vergence.ice
 *
 * \section intro_sec Introduction
* Interface for vergenceComp.  
*                                                                   
* Multiscale head tracking of image regions. 
*
*    PORT 10015 <br>   
*/

/** \namespace RoboCompVergence
  *@brief Name space vergence interface.
  */

module RoboCompVergence
{
  /** \interface Vergence
  *@brief interface Vergence.
  */

  struct coordType {
    int x;
    int y;
  };

  struct sizeType {
    int w;
    int h;
  };

  sequence<coordType> coordList;

  sequence<sizeType> SizeListType;

  sequence<coordList> PositionsListType;


  interface Vergence
  {
    /** @brief vergence fixed check. */
    bool vergenceFixed();
    idempotent void start();
    idempotent void stop();
    bool setVergeAng(float vAng);
    void clearVergeAng();
    bool getVergePosition(float panI, float tiltI, out float panD, out float tiltD);
    void getVergenceData(out RoboCompVision::PyramidType pyramidDom, out RoboCompVision::PyramidType pyramidVerge, out int nLevels, out PositionsListType posList, out SizeListType sizeList);
  };
};

#endif
