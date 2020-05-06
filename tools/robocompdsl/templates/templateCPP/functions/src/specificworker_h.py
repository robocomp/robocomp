import datetime

import dsl_parsers.parsing_utils as p_utils


def agmagent_comment(component):
    result = ""
    try:
        if 'agmagent' in [x.lower() for x in component.options]:
            result +="// THIS IS AN AGENT\n"
    except:
        pass
    return result

def generate_ice_method_params(param, language):
    params_string = ''
    # decorator
    ampersand = '&'
    if param['decorator'] == 'out':
        const = ''
    else:
        if language == "cpp":
            const = 'const '
        else:
            const = ''
            ampersand = ''
        if param['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
            ampersand = ''
    # STR
    params_string += const + param['type'] + ' ' + ampersand + param['name']
    return params_string

def generate_interface_method_definition(component, interface):
    result = ""
    pool = component.idsl_pool
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
                            delim=', '
                        params_string += delim + generate_ice_method_params(param, component.language.lower())


                    result += method['return'] + ' ' + idsl_interface['name'] + "_" + method[
                        'name'] + '(' + params_string + ");\n"
                else:
                    params_string = idsl['name'] + "ROS::" + method['name'] + "::Request &req, " + idsl[
                        'name'] + "ROS::" + method['name'] + "::Response &res"
                    if interface_name in component.iceInterfaces:
                        result += "bool ROS" + method['name'] + '(' + params_string + ");\n"
                    else:
                        result += "bool " + method['name'] + '(' + params_string + ");\n"
    return result

def implements_method_definitions(component):
    result = ""
    for interface in component.implements:
        result += generate_interface_method_definition(component, interface)
    return result

def subscribes_method_definitions(component):
    result = ""
    pool = component.idsl_pool
    for impa in component.subscribesTo:
        if type(impa) == str:
            imp = impa
        else:
            imp = impa[0]
        module = pool.moduleProviding(imp)
        for interface in module['interfaces']:
            if interface['name'] == imp:
                for mname in interface['methods']:
                    method = interface['methods'][mname]
                    paramStrA = ''
                    if p_utils.communication_is_ice(impa):
                        for p in method['params']:
                            # delim
                            if paramStrA == '':
                                delim = ''
                            else:
                                delim = ', '
                            # decorator
                            ampersand = '&'
                            if p['decorator'] == 'out':
                                const = ''
                            else:
                                if component.language.lower() == "cpp":
                                    const = 'const '
                                else:
                                    const = ''
                                    ampersand = ''
                                if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                                    ampersand = ''
                            # STR
                            paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
                        result += method['return'] + ' ' + interface['name'] + "_" + method[
                            'name'] + '(' + paramStrA + ");\n"
                    else:
                        for p in method['params']:
                            # delim
                            if paramStrA == '':
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
                            elif not '::' in p['type']:
                                p['type'] = module['name'] + "ROS::" + p['type']
                            # STR
                            paramStrA += delim + p['type'] + ' ' + p['name']
                        if imp in component.iceInterfaces:
                            result += "void ROS" + method['name'] + '(' + paramStrA + ");\n"
                        else:
                            result += "void " + method['name'] + '(' + paramStrA + ");\n"
    return result

def statemachine_methods(machine):
    result = ""
    if machine['contents']['states'] is not None:
        for state in machine['contents']['states']:
            result += f"void sm_{state}();\n"
    if machine['contents']['initialstate'] is not None:
        result += f"void sm_{machine['contents']['initialstate']}();\n"
    if machine['contents']['finalstate'] is not None:
        result += f"void sm_{machine['contents']['finalstate']}();\n"
    return result

def statemachine_methods_definitions(component):
    result = ""
    sm = component.statemachine
    if component.statemachine_path is not None:
        sm_specification = ""
        sm_specification += statemachine_methods(sm['machine'])
        if sm['substates'] is not None:
            for substates in sm['substates']:
                sm_specification += statemachine_methods(substates)
        result += "//Specification slot methods State Machine\n"
        result += sm_specification+'\n'
        result += "//--------------------\n"
    return result

INNERMODEL_ATTRIBUTES_STR="""\
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
"""

def innermodelviewer_attributes(innermodelviewer):
    result = ""
    if innermodelviewer:
        result += INNERMODEL_ATTRIBUTES_STR
    return result

def agm_attributes(component):
    result = ''
    try:
        if component.is_agm1_agent():
            result += "std::string action;\n"
            result += "ParameterMap params;\n"
            result += "AGMModel::SPtr worldModel;\n"
            result += "bool active;\n"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                result += "void regenerateInnerModelViewer();\n"
            result += "bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);\n"
            result += "void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);\n"
        elif component.is_agm2_agent():
            result += "std::string action;\n"
            result += "AGMModel::SPtr worldModel;\n"
            result += "bool active;\n"
    except:
        pass
    return result

INNERMODELVIEWER_INCLUDES_STR = """\
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif
"""

def innermodelviewer_includes(component):
    result = ""
    if component.innermodelviewer:
        result += INNERMODELVIEWER_INCLUDES_STR
    return result

def constructor_proxies(component):
    result = ""
    if component.language.lower() == 'cpp':
        result += "MapPrx& mprx"
    else:
        result += "TuplePrx tprx"
    return result

def compute(component):
    result = ""
    sm = component.statemachine
    if (sm is not None and sm['machine']['default'] is True) or component.statemachine_path is None:
        result += "void compute();\n"
    return result

def get_template_dict(component):
    return {
        'year': str(datetime.date.today().year),
        'agmagent_comment': agmagent_comment(component),
        'innermodelviewer_includes': innermodelviewer_includes(component),
        'constructor_proxies': constructor_proxies(component),
        'implements_method_definitions': implements_method_definitions(component),
        'subscribes_method_definitions': subscribes_method_definitions(component),
        'compute': compute(component),
        'statemachine_methods_definitions': statemachine_methods_definitions(component),
        'innermodelviewer_attributes': innermodelviewer_attributes(component.innermodelviewer),
        'agm_attributes': agm_attributes(component)
    }