#ifndef ROBOCOMPMOTORS_ICE
#define ROBOCOMPMOTORS_ICE

// #include "common.ice"

module RoboCompMotors{
  	
  struct MotorState{
    int p;
    int v;
    int temperature;
    bool isMoving;
    float pos; 
    float vel;
    float power;
    string timeStamp;
	};

	struct MotorParams{
		bool invertedSign;
		byte busId;
		float minPos;
		float maxPos;
		float maxVelocity;
		float zeroPos;
		float stepsRange;
		float maxDegrees;
		float offset;
		float unitsRange;
		string name;
	};

  struct MotorGoalVelocity{
		float velocity;
		float maxAcc;
		string name;
	};

  struct MotorGoalPosition{
		float position;
		float maxSpeed;
		string name;
	};

  interface Motors
  {
    void setPosition(MotorGoalPosition goal);
    void setVelocity(MotorGoalVelocity goal);
    MotorState getState();
    void setZeroPos();
  };

}; 

#endif //MOTORS_ICE
