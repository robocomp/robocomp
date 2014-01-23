/************************************************************************************************/
/* Interface for components providing access to an IMU 6DOF inertial sensor                     */
/* Possible  sources:                                                                           */
/*                                                                                              */
/*    XSens                                                                                     */
/*                                                                                              */
/************************************************************************************************/
#ifndef IMU_ICE
#define IMU_ICE

module RoboCompIMU
{
	struct Acceleration
	{
		float XAcc;
		float YAcc;
		float ZAcc;
	};
	
	struct Gyroscope
	{
		float XGyr;
		float YGyr;
		float ZGyr;
	};
	
	struct Magnetic
	{
		float XMag;
		float YMag;
		float ZMag;
	};
	
	struct Orientation
	{
		float Roll;
		float Pitch;
		float Yaw;
	};
	
	struct DataImu {
		Acceleration acc;
		Gyroscope gyro;
		Magnetic mag;
		Orientation rot;
		float temperature;
	};
	
	interface IMU
	{
		// Returns IMU linear aceleration, angular velocity, magnetic fields and orientation - euler angles
		DataImu getDataImu();
		
		// Return IMU linear aceleration
		Acceleration getAcceleration();
		
		// Return IMU angular velocity
		Gyroscope getAngularVel();
		
		// Return IMU magnetic field strength
		Magnetic getMagneticFields();
		
		// Return IMU orientation (in euler angles)
		Orientation getOrientation();
		
		// Reset data in Imu Component
		void resetImu();
	};
};

#endif
