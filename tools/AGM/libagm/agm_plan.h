#ifndef AGMJUPLAN_H
#define AGMJUPLAN_H


#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>



typedef std::map< std::string, std::string > AGMParameters;

struct AGMAction
{
public:
	std::string name;
	AGMParameters parameters;

	static AGMAction fromString(std::string actionStr);
};


struct AGMPlan
{
public:
	std::vector<AGMAction> actions;


	static AGMPlan fromString(std::string planStr);

private:

	void append(AGMAction action);

};


#endif
