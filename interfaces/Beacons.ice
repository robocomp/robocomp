#ifndef BEACONS_ICE
#define BEACONS_ICE

module RoboCompBeacons{

        sequence<int> IndexList;        

	interface Beacons
	{
		 void getBeacons(out IndexList indices, out IndexList potencia);
 	};

};
#endif
