#ifndef COMMONBEHAVIOR_ICE
#define COMMONBEHAVIOR_ICE
/** \mainpage RoboComp Interfaces: CommonBehavior.ice
 *
 * \section intro_sec Introduction
* Interface for common behavior
*                                                                   
*                                                                                              
*    PORT 10005 <br>   
*/

/** \namespace RoboCompCommonBehavior
*@brief Name space commonbehavior
*/

module RoboCompCommonBehavior {

	enum State{ Starting, Running};

	/** \struct AttrPair
	*@brief Struct working 
	*/
	struct Parameter
	{
		bool editable;
		string value;
		string type;
	};

	dictionary<string,Parameter> ParameterList;
	
	interface CommonBehavior
	{
		///Get period
		idempotent int getPeriod();
		///Change period
		void setPeriod(int period);
		///Get time awake in miliseconds
		idempotent int timeAwake();
		///Kill component
		void killYourSelf();
		///Get config params
		ParameterList getParameterList();
		///Set config params
		void setParameterList(ParameterList l);
		//Reload config file
		void reloadConfig();
		///Get component state
		State getState();
	};
};

#endif
