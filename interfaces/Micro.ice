#ifndef MICRO_ICE
#define MICRO_ICE

module RoboCompMicro
{
 	sequence<float> sndType;

	interface Micro
	{
		void getData( );
		void getBuffers(out sndType data, out sndType data1 );
	};
};

#endif
