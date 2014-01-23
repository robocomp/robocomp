#ifndef FLOOR_ICE
#define FLOOR_ICE

module RoboCompFloor
{
  struct FloorParams
  {
    int steering;
    int width;
    int height;
    string innerModel;
  };

  sequence<byte> floorMatrixType;

  interface Floor
  {
    void getFloorMatrix(out floorMatrixType floorImage);
    void setNavigation(bool on);
    void goTo(float x, float z);
    bool hasACommand();
    bool navigationEnabled();
    void setTiltControl(bool on);
    bool tiltControlEnabled();
    void setVisible(bool visible);
  };
};

#endif
