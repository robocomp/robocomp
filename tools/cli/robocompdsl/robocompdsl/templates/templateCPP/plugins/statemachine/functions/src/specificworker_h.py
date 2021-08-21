import datetime

import dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict

class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['statemachine_methods_definitions'] = self.statemachine_methods_definitions()


    @staticmethod
    def _statemachine_methods(machine):
        result = ""
        if machine['contents']['states'] is not None:
            for state in machine['contents']['states']:
                result += f"void sm_{state}();\n"
        if machine['contents']['initialstate'] is not None:
            result += f"void sm_{machine['contents']['initialstate']}();\n"
        if machine['contents']['finalstate'] is not None:
            result += f"void sm_{machine['contents']['finalstate']}();\n"
        return result

    def statemachine_methods_definitions(self):
        result = ""
        statemachine = self.component.statemachine
        if self.component.statemachine_path is not None:
            sm_specification = ""
            sm_specification += self._statemachine_methods(statemachine['machine'])
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    sm_specification += self._statemachine_methods(substates)
            result += "//Specification slot methods State Machine\n"
            result += sm_specification+'\n'
            result += "//--------------------\n"
        return result


