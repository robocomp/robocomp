#ifndef MANAGERDEFAULT_ICE
#define MANAGERDEFAULT_ICE

module RoboCompManagerDefault {

  interface ManagerDefault
  {
    idempotent int getFreq();
    void setFreq(out int freq);
    idempotent int timeAwake();
    void killYourSelf();
    string description();
  };
};

#endif
