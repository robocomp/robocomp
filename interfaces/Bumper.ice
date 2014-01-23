#ifndef BUMPER_ICE
#define BUMPER_ICE

module RoboCompBumper {

  struct SensorState {
    int idNumber;
    int state;
  };
  dictionary<string,SensorState> SensorStateMap;

  struct SensorParams{
    string name;
    int idNumber;
    float x;
    float y;
    
  };
  sequence<SensorParams> SensorParamsList;


  interface Bumper {
    void enable();
    void disable();
    bool isEnabled();
    void reset();
    SensorStateMap getSensorData();
  };

};

#endif
