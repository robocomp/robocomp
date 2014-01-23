#ifndef VISION_ICE
#define VISION_ICE

//#include <CommonBehavior.ice>
#include <CommonHead.ice>
#include <DifferentialRobot.ice>
/** \mainpage RoboComp Interfaces: Vision.ice
 *
 * \section intro_sec Introduction
* Interface for visionComp.  
*                                                                   
* Multiscale head tracking of image regions. 
*
*    PORT 10015 <br>   
*/

/** \namespace RoboCompVision
  *@brief Name space vision interface.
  */

module RoboCompVision
{
	/**@brief exception connection*/
  exception ConnectionFailedException{ string what;};
  /**@brief exception hardware*/
  exception HardwareFailedException{ string what;};
	/**@brief exception image*/
  exception MovingImageException { string what; };

  sequence<float> descInfo;

 /** \struct TRoi
  *@brief struct ROI 
  */
  struct TRoi {
    /// Coordinate x in image
    int x; 
    /// Coordinate y in image              
    int y;   
    /// level            
    int i;         
    /// distance to the center      
    int anillo;          
    /// coordinate x in level 0
    int xBase;           
    /// coordinate y in level 0
    int yBase;
    /// coordinate x in level 0 with origin in center of image
    int xBaseCenter;     
    /// coordinate y in level 0 with origin in center of image
    int yBaseCenter;
    /// level in pyramid 0,1,2...
    int level;
    /// octave in pyramid 0,2,4,8,16...
    int scale;
    /// size in level 0 : 2*3*pow(2,scale)
    int size;
    int xHomo;
    int yHomo;
    float x3DW;
    float y3DW;
    float z3DW;
    float x3D;
    float y3D;
    float z3D;
    int xAnt;
    int yAnt;
    bool casado;
    descInfo sift;
  };

  /// vector of roi's
  sequence<TRoi> ROIVector;
  /// vector of vectors or roi's per level
  sequence<ROIVector>ROIPyrVectors;
  /// image	
  sequence<byte> imgType;
  /// pyramid
  sequence<imgType>PyramidType;
  
  struct TState {
    /// See the documentation for CamMotion.ice
    RoboCompCommonHead::THeadState hState;
    /// See the documentation for DifferentialRobot.ice
    RoboCompDifferentialRobot::TBaseState bState;
    string timeStamp;
    string timeStampFormat;
  };
  /** \struct TCamParams
  *@brief struct camera params
  */ 
  struct TCamParams {
    string driver;
    int focal;
    int width;
    int height;
    int prismWidth;
    int prismHeight;
    int size;
    int numCams;
    int FPS;
    int leftCamera;
    int rightCamera;
    int bothCameras;
    int pyrLevels;
    int radius;
    bool colorAvailable;
  };

   /** \interface Vision
  *@brief interface Vision.
  */
   //interface Vision extends RoboCompCommonBehavior::CommonBehavior {
  interface Vision {

    //
    idempotent void getLevelPyr(byte cam, int level, out imgType img, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyr(byte cam, out PyramidType pyr, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;
     
    //
    idempotent void getLevelPyrRGB(byte cam, int level, out imgType img, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrRGB(byte cam, out PyramidType pyr, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getLevelROIList(byte cam, int level, out ROIVector roiList, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholeROIList(byte cam, out ROIPyrVectors roiList, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;
     
    //
    idempotent void getWholePyrAndROIListBothCam(out PyramidType pyrLeft , out ROIPyrVectors roiListLeft,  out PyramidType pyrRight, out ROIPyrVectors roiListRight,  out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrRGBAndROIListBothCam( out PyramidType pyrLeft , out ROIPyrVectors roiListLeft,  out PyramidType pyrRight, out ROIPyrVectors roiListRight,  out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrAndROIList(string cam, out PyramidType pyr , out ROIPyrVectors roiList, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrRGBAndROIList(string cam, out PyramidType pyr , out ROIPyrVectors roiList, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrBothCam(out PyramidType pyrLeft,  out PyramidType pyrRight,  out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getWholePyrRGBBothCam(out PyramidType pyrLeft,  out PyramidType pyrRight,  out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getROILevelPyr(byte cam, int level, int xc, int yc, int w, int h, out imgType img, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;

    //
    idempotent void getROIWholePyr(byte cam, int xc, int yc, int w, int h, out PyramidType pyrRoi, out TState state)
     throws ConnectionFailedException, HardwareFailedException, MovingImageException;


    //
    idempotent TCamParams getCamParams();

    };
};

#endif
