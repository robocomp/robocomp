/*
 * Robolab - RoboComp
 *
 */

#ifndef ROBOCOMP_EXCEPTIONS_ICE
#define ROBOCOMP_EXCEPTIONS_ICE

module RoboCompExceptions
{

	/*!
		@brief A generic run-time exception.
		Components can use this to signal error conditions to their clients.
	*/
	
	exception GenericException
	{
		//! Error description.
		string what;
	};
	
	//! Indicates a problem with robot hardware, e.g. sensors and actuators.
	exception HardwareFailedException extends GenericException {};
	
	//! Indicates a connection problem among components.
	exception ConnectionFailedException extends GenericException {};


}; // module

#endif
