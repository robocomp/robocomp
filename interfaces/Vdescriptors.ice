#ifndef VDESCRIPTORS_ICE
#define VDESCRIPTORS_ICE

#include <CommonHead.ice>
#include <DifferentialRobot.ice>
#include <Roimant.ice>

module RoboCompVDescriptors
{

  sequence<float> descInfo;


  struct TRoiDesc {
    int id;          // Identification number
    int level;
    int casado;
    float x3D;
    float y3D;
    float z3D;
    float pan;
    float tilt;
    descInfo descSpinR;
    descInfo descSpinG;
    descInfo descSpinB;
    descInfo descRift;
  };

  sequence<TRoiDesc> ROIVector;

  interface Vdescriptors
  {

    //
    void getROIDescriptorsInfo(out RoboCompRoimant::ROIVector roiList,
    out RoboCompCommonHead::THeadState hState,
    out RoboCompDifferentialRobot::TBaseState bState);

    //
    void getOneROIDescriptors(int id, out RoboCompRoimant::TRoi roiDesc);

  };
};

#endif
