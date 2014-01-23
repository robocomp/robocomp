#ifndef GROUNDTRUTH2DPOSE_ICE
#define GROUNDTRUTH2DPOSE_ICE
/** \mainpage RoboComp Interfaces: GroundTruth2DPose.ice
 *
 * \section intro_sec Introduction
* Interface for 2D pose ground-truth.
*                                                                   
* Provides the ground-truth pose of a robot within a global plane.
*                                                                                              
*/

module RoboCompGroundTruth2DPose
{
  struct GTPose2D
  {
    float x; 
    float z;    
    float alpha;
  };

  interface GroundTruth2DPose
  {
    /// Get pose
    void getPose(out GTPose2D pose);
  };
};

#endif
