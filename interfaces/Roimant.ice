#ifndef ROIMANT_ICE
#define ROIMANT_ICE

#include <Vision.ice>
/** \mainpage RoboComp Interfaces: Roimant.ice
 *
 * \section intro_sec Introduction
 * Interface for roimantComp.  
 *                                                                   
 * Tracking of regions of interest detector (ROIs) in image space.<br>
 *
 *    PORT 10013 <br>   
 */

/** \namespace RoboCompRoimant
  * @brief Name space roimant.
  */

module RoboCompRoimant
{
  /** @brief exception connection*/
  exception ConnectionFailedException{ string what; };

  /// Image type
  sequence<byte> imgType;
  /// pyramid
  sequence<imgType>PyramidType;
  /// Image
  sequence<byte> ImgReg;
  sequence<int> pointSeq;
  sequence<int> IntSeq;
  sequence<float> descInfo;
  
  struct T3DCoor {
    bool avalaible;
    float x3D;
    float y3D;
    float z3D;
  };
  sequence<T3DCoor> T3DCoorSequence;

  /** \struct TRoi
  *@brief struct ROI 
  */
  struct TRoi {
    /// Identification number
    int id;           
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
    /// distance to center of image in its level
    float distcentro;
    ///
    float pan;
    ///
    float tilt;
    ///
    float panHomo;
    ///
    float tiltHomo;
    ///
    float x3D;
    ///
    float y3D;
    ///
    float z3D;
    ///
    float x3DW;
    ///
    float y3DW;
    ///
    float z3DW;
    ///
    T3DCoorSequence p3Dsequence;
    ///
    int xHomo;
    ///
    int yHomo;
    ///
    int xAnt;
    ///
    int yAnt;
    ///
    int tracked;
    ///
    int casado;
    ///
    int permanencia;
    ///
    float atencion;
    ///
    ImgReg img;
    ///
    int act;
    ///
    int accion;
    /// x speed of regions in image coordinates: pixels-base/refresh-period
    float vx;           
    /// y speed of regions in image coordinates: pixels-base/refresh-period
    float vy;
    /// elapsed refresh interval in msecs
    int interval;       
    /// msecs alive
    int age;            
    /// incremental changes in position
    pointSeq trayecX;   
    /// incremental changes in position
    pointSeq trayecY;   
    int suelo;
    int descriptors;
    descInfo descSpinR;
    descInfo descSpinG;
    descInfo descSpinB;
    descInfo descRift;
    descInfo sift;
    ImgReg imgExt;
  };

  /** \struct TCamParams
  *@brief struct camera params 
  * Holds which camera is dominant and which is slave in each iteration.
  */
  struct TCamParams {
    byte dominant;
    byte vergence;
  };

  /** \struct TRoiParams
  *@brief struct ROI params 
  */
  struct TRoiParams {
    // Width of the 0 level of the pyramid
    int width;
    // Height of the 0 level of the pyramid
    int height;
    int wImgExt;
    int hImgExt;
    string robotName;
  };

  /** \struct TState
  *@brief struct state params 
  */
  struct TState {
    /// See the documentation for CommonHead.ice
    RoboCompCommonHead::THeadState hState;
    /// See the documentation for DifferentialRobot.ice
    RoboCompDifferentialRobot::TBaseState bState;
    string timeStamp;
    ///
    string timeStampFormat;
  };
   
  /// vector of roi's
  sequence<TRoi> ROIVector;

   /** \interface Roimant
  *@brief interface Roimant.
  */  
  interface Roimant {
    // Provides the ROI vector from the dominant camera with the head and base state.
    idempotent void getROIList(out ROIVector roiList, out TState state);
    // Provides the ROI vector from the dominant camera with the head and base state and the index of the current iteration.
    idempotent void getROIListAndCurrentIteration(out ROIVector roiList, out TState state, out int currentIter);
    // Provides the ROI vector and pyramid image of the slave camera, with the head and base state.
    idempotent void getROIListAndVergencePyr(out ROIVector roiList, out PyramidType pyr, out TState state);
    // Provides both image pyramids in gray scale, the ROI vector and the head and base state.
    void getBothPyramidsAndLeftROIList(out PyramidType pirLeft,  out ROIVector roiListLeft, out PyramidType pirRight, out TState state);
    // Provides both image pyramids in RGB, the ROI vector and the head and base state.
    void getBothPyramidsRGBAndLeftROIList(out PyramidType pirLeft,  out ROIVector roiListLeft, out PyramidType pirRight, out TState state);
    //
    void setROIListDescriptors(ROIVector roiList);
    //
    TCamParams getCamParams();
    //
    TRoiParams getRoiParams();
  };
};

#endif
