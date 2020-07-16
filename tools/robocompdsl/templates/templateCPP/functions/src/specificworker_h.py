import datetime

import dsl_parsers.parsing_utils as p_utils
from .. import function_utils as utils

INNERMODEL_ATTRIBUTES_STR = """\
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
"""


INNERMODELVIEWER_INCLUDES_STR = """\
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif
"""

DSR_INCLUDES_STR = """\
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
"""

DSR_ATTRIBUTES = """\
// DSR graph
std::shared_ptr<DSR::DSRGraph> G;

//DSR params
std::string agent_name;
int agent_id;

bool tree_view;
bool graph_view;
bool qscene_2d_view;
bool osg_3d_view;

// DSR graph viewer
std::unique_ptr<DSR::GraphViewer> graph_viewer;
QHBoxLayout mainLayout;
"""

class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['agmagent_comment'] = self.agmagent_comment()
        self['innermodelviewer_includes'] = self.innermodelviewer_includes()
        self['constructor_proxies'] = self.constructor_proxies()
        self['implements_method_definitions'] = self.implements_method_definitions()
        self['subscribes_method_definitions'] = self.subscribes_method_definitions()
        self['compute'] = self.compute()
        self['statemachine_methods_definitions'] = self.statemachine_methods_definitions()
        self['innermodelviewer_attributes'] = self.innermodelviewer_attributes()
        self['agm_attributes'] = self.agm_attributes()
        self['dsr_includes'] = self.dsr_includes()
        self['dsr_attributes'] = self.dsr_attributes()

    def agmagent_comment(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += "// THIS IS AN AGENT\n"
        return result

    def generate_interface_method_definition(self, interface):
        result = ""
        pool = self.component.idsl_pool
        if type(interface) == str:
            interface_name = interface
        else:
            interface_name = interface.name
        module = pool.module_providing_interface(interface_name)
        for idsl_interface in module['interfaces']:
            if idsl_interface['name'] == interface_name:
                for method_name, method in idsl_interface['methods'].items():
                    if p_utils.communication_is_ice(interface):
                        params_string = utils.get_parameters_string(method, module['name'], self.component.language)
                        return_type = utils.get_type_string(method['return'], module['name'])
                        result += return_type + ' ' + idsl_interface['name'] + "_" + method[
                            'name'] + '(' + params_string + ");\n"
                    else:
                        pass
        return result

    def implements_method_definitions(self):
        result = ""
        for interface in self.component.implements:
            result += self.generate_interface_method_definition(interface)
        return result

    def subscribes_method_definitions(self):
        result = ""
        pool = self.component.idsl_pool
        for impa in self.component.subscribesTo:
            if type(impa) == str:
                imp = impa
            else:
                imp = impa.name
            module = pool.module_providing_interface(imp)
            for interface in module['interfaces']:
                if interface['name'] == imp:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        if p_utils.communication_is_ice(impa):
                            param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                            return_type = utils.get_type_string(method['return'], module['name'])
                            result += return_type + ' ' + interface['name'] + "_" + method[
                                'name'] + '(' + param_str_a + ");\n"
                        else:
                            pass
        return result

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

    def innermodelviewer_attributes(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODEL_ATTRIBUTES_STR
        return result

    def agm_attributes(self):
        result = ''
        if self.component.is_agm1_agent():
            result += "std::string action;\n"
            result += "RoboCompAGMCommonBehavior::ParameterMap params;\n"
            result += "AGMModel::SPtr worldModel;\n"
            result += "bool active;\n"
            if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                result += "void regenerateInnerModelViewer();\n"
            result += "bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);\n"
            result += "void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);\n"
        elif self.component.is_agm2_agent():
            result += "std::string action;\n"
            result += "AGMModel::SPtr worldModel;\n"
            result += "bool active;\n"
        return result

    def innermodelviewer_includes(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODELVIEWER_INCLUDES_STR
        return result

    def dsr_includes(self):
        result = ""
        if self.component.dsr:
            result = DSR_INCLUDES_STR
        return result


    def constructor_proxies(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "MapPrx& mprx"
        else:
            result += "TuplePrx tprx"
        return result

    def compute(self):
        result = ""
        sm = self.component.statemachine
        if (sm is not None and sm['machine']['default'] is True) or self.component.statemachine_path is None:
            result += "void compute();\n"
        return result

    def dsr_attributes(self):
        result=""
        if self.component.dsr:
            result = DSR_ATTRIBUTES
        return result


