#ifndef RCREMOTE_ICE
#define RCREMOTE_ICE

module RoboCompRemote
{
	sequence <string> stringList;

	interface RCRemote
	{
		bool run(string randomstuff, string stuffPasswordHashed, string path, string binary, stringList arguments, string yakuakeTabName);
	};
};

#endif
