/*
 * Dependences: Laser
 */
#ifndef LOCALNAVIGATOR_ICE
#define LOCALNAVIGATOR_ICE

module RoboCompLocalNavigator
{
  struct Stage
  {
      int x;
      int z;
  };
  sequence<Stage> Trajectory;
	
	interface LocalNavigator
  {
    void stop();
	 bool isActive();
    bool goTo(float x, float z);
    void setPathVel(Trajectory path, float advVel, float rotVel);
  };
};

#endif
