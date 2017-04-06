#ifndef ROBOCOMPTRIPODCONTROLLER_ICE
#define ROBOCOMPTRIPODCONTROLLER_ICE

module RoboCompTripodController
{
	exception HardwareFailedException{string  what;};
	
	struct State
	{
		int a;
		string st;
	};
	interface TripodController
	{
		State getState() throws HardwareFailedException;
		void action(string act) throws HardwareFailedException;
	};
};
#endif
