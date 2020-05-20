import datetime

import dsl_parsers.parsing_utils as p_utils


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

    def agmagent_comment(self):
        result = ""
        try:
            if 'agmagent' in [x.lower() for x in self.component.options]:
                result += "// THIS IS AN AGENT\n"
        except:
            pass
        return result

    def generate_ice_method_params(self, param):
        params_string = ''
        # decorator
        ampersand = '&'
        if param['decorator'] == 'out':
            const = ''
        else:
            if self.component.language.lower() == "cpp":
                const = 'const '
            else:
                const = ''
                ampersand = ''
            if param['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                ampersand = ''
        # STR
        params_string += const + param['type'] + ' ' + ampersand + param['name']
        return params_string

    def generate_interface_method_definition(self, interface):
        result = ""
        pool = self.component.idsl_pool
        if type(interface) == str:
            interface_name = interface
        else:
            interface_name = interface[0]
        idsl = pool.moduleProviding(interface_name)
        for idsl_interface in idsl['interfaces']:
            if idsl_interface['name'] == interface_name:
                for method_name, method in idsl_interface['methods'].items():
                    params_string = ''
                    if p_utils.communication_is_ice(interface):
                        delim = ''
                        for index, param in enumerate(method['params']):
                            if index != 0:
                                delim = ', '
                            params_string += delim + self.generate_ice_method_params(param)

                        result += method['return'] + ' ' + idsl_interface['name'] + "_" + method[
                            'name'] + '(' + params_string + ");\n"
                    else:
                        params_string = idsl['name'] + "ROS::" + method['name'] + "::Request &req, " + idsl[
                            'name'] + "ROS::" + method['name'] + "::Response &res"
                        if interface_name in self.component.iceInterfaces:
                            result += "bool ROS" + method['name'] + '(' + params_string + ");\n"
                        else:
                            result += "bool " + method['name'] + '(' + params_string + ");\n"
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
                imp = impa[0]
            module = pool.moduleProviding(imp)
            for interface in module['interfaces']:
                if interface['name'] == imp:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        if p_utils.communication_is_ice(impa):
                            for p in method['params']:
                                # delim
                                if param_str_a == '':
                                    delim = ''
                                else:
                                    delim = ', '
                                # decorator
                                ampersand = '&'
                                if p['decorator'] == 'out':
                                    const = ''
                                else:
                                    if self.component.language.lower() == "cpp":
                                        const = 'const '
                                    else:
                                        const = ''
                                        ampersand = ''
                                    if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                                        ampersand = ''
                                # STR
                                param_str_a += delim + const + p['type'] + ' ' + ampersand + p['name']
                            result += method['return'] + ' ' + interface['name'] + "_" + method[
                                'name'] + '(' + param_str_a + ");\n"
                        else:
                            for p in method['params']:
                                # delim
                                if param_str_a == '':
                                    delim = ''
                                else:
                                    delim = ', '
                                # decorator
                                ampersand = '&'
                                if p['decorator'] == 'out':
                                    const = ''
                                else:
                                    const = 'const '
                                    ampersand = ''
                                if p['type'] in ('float', 'int'):
                                    p['type'] = "std_msgs::" + p['type'].capitalize() + "32"
                                elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                                    p['type'] = "std_msgs::UInt" + p['type'].split('t')[1]
                                elif p['type'] in pool.getRosTypes():
                                    p['type'] = "std_msgs::" + p['type'].capitalize()
                                elif '::' not in p['type']:
                                    p['type'] = module['name'] + "ROS::" + p['type']
                                # STR
                                param_str_a += delim + p['type'] + ' ' + p['name']
                            if imp in self.component.iceInterfaces:
                                result += "void ROS" + method['name'] + '(' + param_str_a + ");\n"
                            else:
                                result += "void " + method['name'] + '(' + param_str_a + ");\n"
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
        try:
            if self.component.is_agm1_agent():
                result += "std::string action;\n"
                result += "ParameterMap params;\n"
                result += "AGMModel::SPtr worldModel;\n"
                result += "bool active;\n"
                if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                    result += "void regenerateInnerModelViewer();\n"
                result += "bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);\n"
                result += "void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);\n"
            elif self.component.is_agm2_agent():
                result += "std::string action;\n"
                result += "AGMModel::SPtr worldModel;\n"
                result += "bool active;\n"
        except:
            pass
        return result

    def innermodelviewer_includes(self):
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
