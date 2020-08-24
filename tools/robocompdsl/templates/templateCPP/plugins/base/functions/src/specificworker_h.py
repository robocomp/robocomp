import datetime

import dsl_parsers.parsing_utils as p_utils
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

INNERMODELVIEWER_ATTRIBUTES_STR = """\
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
"""

INNERMODEL_ATTRIBUTE_STR = """\
std::shared_ptr < InnerModel > innerModel;
"""


INNERMODEL_INCLUDE_STR = """\
#include <innermodel/innermodel.h>
"""

INNERMODELVIEWER_INCLUDES_STR = """\
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif
"""

class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['innermodel_include'] = self.innermodel_include()
        self['innermodelviewer_includes'] = self.innermodelviewer_includes()
        self['constructor_proxies'] = self.constructor_proxies()
        self['implements_method_definitions'] = self.implements_method_definitions()
        self['subscribes_method_definitions'] = self.subscribes_method_definitions()
        self['compute'] = self.compute()
        self['statemachine_methods_definitions'] = self.statemachine_methods_definitions()
        self['innermodel_attribute'] = self.innermodel_attribute()
        self['innermodelviewer_attributes'] = self.innermodelviewer_attributes()


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


    def innermodel_attribute(self):
        result = ""
        if "dsr" not in self.component.options:
            result += INNERMODEL_ATTRIBUTE_STR
        return result

    def innermodelviewer_attributes(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODELVIEWER_ATTRIBUTES_STR
        return result


    def innermodelviewer_includes(self):
        result = ""
        if "dsr" not in self.component.options:
            result += INNERMODEL_INCLUDE_STR
        return result

    def innermodel_include(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODELVIEWER_INCLUDES_STR
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


