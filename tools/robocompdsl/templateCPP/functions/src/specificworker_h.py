import dsl_parsers.parsing_utils as p_utils


def generate_ice_method_params(param, language):
    params_string = ''
    # delim
    if params_string == '':
        delim = ''
    else:
        delim = ', '
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
    params_string += delim + const + param['type'] + ' ' + ampersand + param['name']
    return params_string

def generate_interface_method_definition(component, interface, pool):
    result = ""
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
                    for param in method['params']:
                        params_string += generate_ice_method_params(param, component.language.lower())
                    result += "<TABHERE>" + method['return'] + ' ' + idsl_interface['name'] + "_" + method[
                        'name'] + '(' + params_string + ");\n"
                else:
                    params_string = idsl['name'] + "ROS::" + method['name'] + "::Request &req, " + idsl[
                        'name'] + "ROS::" + method['name'] + "::Response &res"
                    if interface_name in component.iceInterfaces:
                        result += "<TABHERE>bool ROS" + method['name'] + '(' + params_string + ");\n"
                    else:
                        result += "<TABHERE>bool " + method['name'] + '(' + params_string + ");\n"
    return result

def specificworker_implements_method_definitions(pool, component):
    result = ""
    if 'implements' in component:
        for interface in component.implements:
            result += generate_interface_method_definition(component, interface, pool)
    return result

def specificworker_subscribes_method_definitions(pool, component):
    result = ""
    if 'subscribesTo' in component:
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
                            result += "<TABHERE>" + method['return'] + ' ' + interface['name'] + "_" + method[
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
                                result += "<TABHERE>void ROS" + method['name'] + '(' + paramStrA + ");\n"
                            else:
                                result += "<TABHERE>void " + method['name'] + '(' + paramStrA + ");\n"
    return result

def statemachine_methods(machine):
    result = ""
    if machine['contents']['states'] is not None:
        for state in machine['contents']['states']:
            result += f"<TABHERE>void sm_{state}();\n"
    if machine['contents']['initialstate'] is not None:
        result += f"<TABHERE>void sm_{machine['contents']['initialstate']}();\n"
    if machine['contents']['finalstate'] is not None:
        result += f"<TABHERE>void sm_{machine['contents']['finalstate']}();\n"
    return result

def specificworker_statemachine_methods_definitions(component, sm):
    result = ""
    if component.statemachine is not None:
        sm_specification = ""
        sm_specification += statemachine_methods(sm['machine'])
        if sm['substates'] is not None:
            for substates in sm['substates']:
                sm_specification += statemachine_methods(substates)
        result += "//Specification slot methods State Machine\n"
        result += sm_specification+'\n'
        result += "//--------------------\n"
    return result

def specificworker_innermodelviewer_attributes(innermodelviewer):
    result = ""
    if innermodelviewer:
        result +="#ifdef USE_QTGUI\n"
        result +="<TABHERE>OsgView *osgView;\n"
        result +="<TABHERE>InnerModelViewer *innerModelViewer;\n"
        result +="#endif\n"
    return result

def specificworker_agm_attributes(component):
    result = ''
    try:
        if p_utils.is_agm1_agent(component):
            result += "<TABHERE>std::string action;\n"
            result += "<TABHERE>ParameterMap params;\n"
            result += "<TABHERE>AGMModel::SPtr worldModel;\n"
            result += "<TABHERE>bool active;\n"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                result += "<TABHERE>void regenerateInnerModelViewer();\n"
            result += "<TABHERE>bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);\n"
            result += "<TABHERE>void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);\n"
        elif p_utils.is_agm2_agent(component):
            result += "<TABHERE>std::string action;\n"
            result += "<TABHERE>AGMModel::SPtr worldModel;\n"
            result += "<TABHERE>bool active;\n"
    except:
        pass
    return result