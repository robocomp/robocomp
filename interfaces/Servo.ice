#ifndef SERVO_ICE
#define SERVO_ICE

module RoboCompServo
{
	exception HardwareFailedException{ string what; };
	exception UnknownSensorException{ string what; };

	interface Servo
	{
		void setAngleServo(float angle)throws RoboCompServo::HardwareFailedException;
		float getAngleServo()throws RoboCompServo::HardwareFailedException;
	};
};

#endif
