#ifndef ROBOCOMPGENERICBASE_ICE
#define ROBOCOMPGENERICBASE_ICE

module RoboCompGenericBase{
	exception HardwareFailedException{string what;};
	
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
	  
	interface GenericBase
	{
		void getBaseState(out RoboCompGenericBase::TBaseState state) throws HardwareFailedException;
		void getBasePose(out int x, out int z, out float alpha) throws  HardwareFailedException;
	};
};
  
#endif