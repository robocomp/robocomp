#ifndef BOTTOMUPSELECTOR_ICE
#define BOTTOMUPSELECTOR_ICE

module RoboCompBottomUpSelector
{

	interface BottomUpSelector
	{
		void startAttention();
		void stopAttention();
		void setSearchParams(float pAtt, float tAtt, float minPan, float maxPan, float minTilt, float maxTilt, int level);
		void reset();
		int getGlobalSelectorId();
	};
};

#endif
