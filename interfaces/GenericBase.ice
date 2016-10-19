#ifndef ROBOCOMPGENERICBASE_ICE
#define ROBOCOMPGENERICBASE_ICE

module RoboCompGenericBase{
	["cpp:comparable"]
	struct TBaseState{
		bool isMoving;
		float x;
		float z;
		float alpha;
		float correctedX;
		float correctedZ;
		float correctedAlpha;
		float advVx;
		float advVz;
		float rotV;
	};
};
  
#endif