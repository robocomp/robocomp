#ifndef ROBOCOMPAGMCOMMONBEHAVIOR_ICE
#define ROBOCOMPAGMCOMMONBEHAVIOR_ICE

module RoboCompAGMCommonBehavior{

	enum StateEnum { Starting, Running, Stopped };

	["cpp:comparable"]
	struct StateStruct
	{
		StateEnum state;
		string info;
	};

	["cpp:comparable"]
	struct Parameter
	{
		bool editable;
		string value;
		string type;
	};

	dictionary<string, Parameter>ParameterMap;

	interface AGMCommonBehavior
	{
		bool activateAgent(ParameterMap prs);
		bool deactivateAgent();
		StateStruct getAgentState();
		ParameterMap getAgentParameters();
		bool setAgentParameters(ParameterMap prs);
		void killAgent();
		int uptimeAgent();
		bool reloadConfigAgent();
	};
};
  
#endif