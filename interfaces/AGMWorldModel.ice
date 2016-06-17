#ifndef ROBOCOMPAGMWORLDMODEL_ICE
#define ROBOCOMPAGMWORLDMODEL_ICE

module RoboCompAGMWorldModel
{
	dictionary<string, string> StringDictionary;

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
		NodeSequence nodes;
		EdgeSequence edges;
		int version;
	};

};
  
#endif
