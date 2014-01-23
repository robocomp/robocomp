#ifndef TRACKER_ICE
#define TRACKER_ICE

#include <Vision.ice>

/** \mainpage RoboComp Interfaces: Tracker.ice
 *
 * \section intro_sec Introduction
* Interface for trackerComp.  
*                                                                   
* Multiscale head tracking of image regions. 
*
*    PORT 10009 <br>   
*/

/** \namespace RoboCompTracker
  *@brief Name space tracker interface.
  */

module RoboCompTracker
{

  sequence<byte> imgType;

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
  
  enum TrackerState{NoTarget, LostTarget, Tracking};
 

  /** \interface Tracker
  *@brief interface Tracker.
  */
    interface Tracker
    {
    /**@brief Track to image coordinates in some pyramid level that fizates the width of the region */
    void trackCoor(int x, int y, int level);

    /**@brief Track to image window */
    void trackWin(int x, int y, int w, int h);

    /**@brief Track to Region of interest computed by RoimantComp */
    void trackRoi(int roiId, byte cam);

    /**@brief Track to Angular coordinates head reference frame */
    void trackAng(float pan, float tilt, int level);

    /**@brief Track to Angular coordinates head reference frame specifying the dimensions of the target region */
    void trackAngWithDim(float pan, float tilt, int w, int h);

    /**@brief Track to 3D Angular coordinates head reference frame */
    void track3DAng(float pan, float tilt, float panVerg, int level);

    void updateDistToTarget(float dist);

    /**@brief Return current tracked coordinates */
    int getTrackCoor(out int x, out int y, out int w, out int h);

    /**@brief Stop tracking */
    void trackStop();

    void getTrackingData(out RoboCompVision::PyramidType pyramid, out int nLevels, out PositionsListType posList, out SizeListType sizeList);
    
    TrackerState getTrackerState();
    
    };
};

#endif
