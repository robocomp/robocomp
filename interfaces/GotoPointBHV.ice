#ifndef GOTOPOINTBHV_ICE
#define GOTOPOINTBHV_ICE

#include <Roimant.ice>

module RoboCompGotoPointBHV
{

	sequence<float> MapType;

	interface GotoPointBHV
	{
		void updateRoi(RoboCompRoimant::TRoi roi);
		void set3DRef(float x3D, float y3D, float z3D);
		void startControl();
		void stopControl();
		void resumeControl();
		void pauseControl();
		void getOccupancyMap(out MapType OCCMap, out int wMap, out int hMap, out float sizeReg);
		
	};
};

#endif
