#ifndef NAVIGATORBHV_ICE
#define NAVIGATORBHV_ICE

module RoboCompNavigatorBHV
{

	interface NavigatorBHV
	{
		void set3DRef(float x3D, float y3D, float z3D);
		void startControl();
		void stopControl();
	};
};

#endif
