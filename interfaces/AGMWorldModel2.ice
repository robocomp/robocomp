#ifndef ROBOCOMPAGMWORLDMODEL2_ICE
#define ROBOCOMPAGMWORLDMODEL2_ICE

module RoboCompAGMWorldModel
{

	dictionary<string, string>StringDictionary;

	["cpp:comparable"]
	struct Node
	{
		StringDictionary attributes;
		int nodeIdentifier;
		string nodeType;
	};

	sequence <Node> NodeSequence;
	
	["cpp:comparable"]
	struct Edge
	{
		StringDictionary attributes;
		int a;
		int b;
		string edgeType;
	};

	sequence <Edge> EdgeSequence;

	["cpp:comparable"]
	struct World
	{
		int version;
		NodeSequence nodes;
		EdgeSequence edges;
	};

};
  
#endif
