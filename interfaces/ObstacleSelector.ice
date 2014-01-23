#ifndef OBSTACLESELECTOR_ICE
#define OBSTACLESELECTOR_ICE

module RoboCompObstacleSelector
{
  exception ConnectionFailedException{ string what;};

  sequence<byte> imgType;

  interface ObstacleSelector
  {
    void gotoDir(float pan, float tilt, int level);
    void gotoDir3D(float x3D, float y3D, float z3D);
    void gotoROI(int roiId);
    void startAttention();
    void stopAttention();
    void sinRumbo();
    void getImageSal(out imgType img, out int xAtt, out int yAtt);
    void getRef3D(out float x3D, out float y3D, out float z3D);
    int getGlobalSelectorId();
  };
};

#endif
