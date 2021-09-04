import datetime
from string import Template

import robocompdsl.dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict

AGM_INNERMODEL_ASSOCIATION_STR = """\
innerModel = std::make_shared<InnerModel>(new InnerModel());
try
{
	RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
	AGMExecutiveTopic_structuralChange(w);
}
catch(...)
{
	printf("The executive is probably not running, waiting for first AGM model publication...");
}
"""

SET_PARAMETERS_AND_POSSIBLE_ACTIVATION = """
bool SpecificWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\\n");

	return true;
}
"""

SEND_MODIFICATION_PROPOSAL = """
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, ${proxy}, \"${agent_name}Agent\");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}
"""

class specificworker_cpp(TemplateDict):
    def __init__(self, component):
        super(specificworker_cpp, self).__init__()
        self.component = component
        self['agmagent_attributes'] = self.agmagent_attributes()
        self['agm_innermodel_association'] = self.agm_innermodel_association()
        self['agm_specific_code'] = self.agm_specific_code()


    def agmagent_attributes(self):
        result = ""
        if self.component.is_agm_agent():
            result += "active = false;\n"
            result += "worldModel = AGMModel::SPtr(new AGMModel());\n"
            result += "worldModel->name = \"worldModel\";\n"
        return result

    def agm_innermodel_association(self):
        result = ""
        if self.component.is_agm_agent():
            result += AGM_INNERMODEL_ASSOCIATION_STR
        return result


    def agm_specific_code(self):
        result = ""
        if ('agmagent' in [x.lower() for x in self.component.options]) and (
                'innermodelviewer' in [x.lower() for x in self.component.options]):
            result += REGENERATE_INNERMODEL

        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += SET_PARAMETERS_AND_POSSIBLE_ACTIVATION
            agent_name = self.component.name
            if self.component.language.lower() == "cpp":
                proxy = "agmexecutive_proxy"
            else:
                proxy = "*agmexecutive_proxy.get()"
            result += Template(SEND_MODIFICATION_PROPOSAL).substitute(proxy=proxy, agent_name=agent_name)
        return result