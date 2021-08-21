import datetime

import dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict


class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['agmagent_comment'] = self.agmagent_comment()
        self['agm_attributes'] = self.agm_attributes()

    def agmagent_comment(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += "// THIS IS AN AGENT\n"
        return result

    def agm_attributes(self):
        result = ''
        if self.component.is_agm_agent():
            result += "std::string action;\n"
            result += "RoboCompAGMCommonBehavior::ParameterMap params;\n"
            result += "AGMModel::SPtr worldModel;\n"
            result += "bool active;\n"
            if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                result += "void regenerateInnerModelViewer();\n"
            result += "bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);\n"
            result += "void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);\n"
        return result


