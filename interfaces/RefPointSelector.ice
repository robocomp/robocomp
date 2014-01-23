#ifndef REFPOINTSELECTOR_ICE
#define REFPOINTSELECTOR_ICE

module RoboCompRefPointSelector
{

	interface RefPointSelector
	{
		void startAttention();
		void stopAttention();
		void fixateSearchZone(float x3D, float y3D, float z3D, bool useDist);
		void deleteSearchZone();
		int getGlobalSelectorId();
		void setSearchAng(float angSearch);
	};
};

#endif
