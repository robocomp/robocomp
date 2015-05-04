/************************************************************************************************/
/* Interface for components providing access to an IMU 6DOF inertial sensor                     */
/* Possible  sources:                                                                           */
/*                                                                                              */
/*    XSens                                                                                     */
/*                                                                                              */
/************************************************************************************************/
#ifndef IMUPub_ICE
#define IMUPub_ICE

module RoboCompIMUPub
{
	struct QuatRotation{
		float w;
		float x;
		float y;
		float z;
		 
		 };
	

	interface IMUPub
	{
		// Returns orientation
		void setOrientation(QuatRotation myquat);
		
	};
};

#endif
