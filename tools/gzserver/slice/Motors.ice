#ifndef ROBOCOMPMOTORS_ICE
#define ROBOCOMPMOTORS_ICE

// #include "common.ice"

module RoboCompMotors{

  interface Motors
  {
    float getMotorSpeed();
    void setMotorSpeed(float w);
  };

}; 

#endif //MOTORS_ICE
