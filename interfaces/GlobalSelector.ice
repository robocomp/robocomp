#ifndef GLOBALSELECTOR_ICE
#define GLOBALSELECTOR_ICE

#include <Roimant.ice>

module RoboCompGlobalSelector
{

 enum TRoiType { NoRoi, SpatialRoi, VisualRoi };
 
 struct TAttendedRoi
 {
    TRoiType roiType;
    int level;
    bool avalaible3D;
    //3D in world coordinates
    float x3D;
    float y3D;
    float z3D;
    
    float panI;
    float tiltI;

    bool blindSaccadic;
    float tilt;
    float leftPan;
    float rightPan;
    float neck;
    
  };
 
 struct TSelectorsData
 {
  int id;
  float attentionTime;
  float alertLevel;
 };


 interface GlobalSelector
 {
  int addTargetSelector(TSelectorsData tsData);
  void deleteTargetSelector(int id);
  void deactivateTargetSelector(int id);
  void changeSelectorData(TSelectorsData tsData);
  void attentionOnRoi(int idTargetSelector, TAttendedRoi roi);
  int getAttendedRoi(out TAttendedRoi roi, out int idSelector);
  int cambiaTilt();
  int setRadSaccadic(float pan, float tilt);
  int set3DSaccadic(float x3D, float y3D, float z3D);
  int isSaccadicReachable(float x3D, float y3D, float z3D);
 };
};

#endif
