import datetime
from string import Template

from robocompdsl.templates.common.templatedict import TemplateDict


AGM_BEHAVIOUR_STRUCT_STR = """
struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};
"""


AGM_ATTRIBUTES_STR = """
bool active;
AGMModel::SPtr worldModel;
BehaviorParameters p;
RoboCompAGMCommonBehavior::ParameterMap params;
int iter;
bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
RoboCompPlanning::Action createAction(std::string s);
"""

class genericworker_h(TemplateDict):

    def __init__(self, component):
        super(genericworker_h, self).__init__()
        self.component = component
        self['agm_includes'] = self.agm_includes()
        self['agm_behaviour_parameter_struct'] = self.agm_behaviour_parameter_struct()
        self['agm_methods'] = self.agm_methods()
        self['agm_attributes_creation'] = self.agm_attributes_creation()

    def agm_includes(self):
        result = ""
        if self.component.is_agm_agent():
            result += "#include <agm.h>\n"
        return result

    def agm_behaviour_parameter_struct(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_BEHAVIOUR_STRUCT_STR
        return result

    def agm_methods(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += "bool activate(const BehaviorParameters& parameters);\n"
            result += "bool deactivate();\n"
            result += "bool isActive() { return active; }\n"
        return result

    def agm_attributes_creation(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_ATTRIBUTES_STR
        return result
