#ifndef GOTOPOINT_ICE
#define GOTOPOINT_ICE

#include <Cube.ice>

module RoboCompGotoPoint
{

  enum GotoPointState{waitingForTarget, targetReached, unreachableTarget, movingToTarget};

  interface GotoPoint
  {
    void set3DRef(float x3D, float y3D, float z3D);
    void startControl();
    void stopControl();
    void resumeControl();
    void pauseControl();
    bool getCurrentPath(out RoboCompCube::Point3DList path);
    GotoPointState getState();
  };
};

#endif
