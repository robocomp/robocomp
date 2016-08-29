#include <agm_plan.h>


AGMAction AGMAction::fromString(std::string actionStr)
{
	AGMAction action;

	std::vector<std::string> actionStrs;
	boost::split(actionStrs, actionStr, boost::is_any_of("\n@,{}"), boost::token_compress_on);
	action.name = actionStrs[0];
// 	printf("Action: <%s>\n", action.name.c_str());
	for (uint i=1; i<actionStrs.size(); i++)
	{
		std::string parameter = actionStrs[i];
		std::vector<std::string> pairStrs;
		boost::split(pairStrs, parameter, boost::is_any_of("\n@,{}\t :'\""), boost::token_compress_on);
		pairStrs.erase(std::remove(pairStrs.begin(), pairStrs.end(), ""), pairStrs.end());
		if (pairStrs.size()>1)
		{
// 			printf("  %d<%s#%s>\n", (int)pairStrs.size(), pairStrs[0].c_str(), pairStrs[1].c_str());
			action.parameters[pairStrs[0]] = pairStrs[1];
		}
	}
	
	return action;
}

AGMPlan AGMPlan::fromString(std::string planStr)
{
	AGMPlan plan;

	std::vector<std::string> actionStrs;
	boost::split(actionStrs, planStr, boost::is_any_of("\n"));

	for (auto actionStr : actionStrs)
	{
		if (actionStr.size() > 0)
		{
			plan.append(AGMAction::fromString(actionStr));
		}
	}
	
	return plan;
}

void AGMPlan::append(AGMAction action)
{
	actions.push_back(action);
}


