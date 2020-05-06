from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent, IDSLPool, get_name_number

GUI_INCLUDE_STR = """
#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
"""


def gui_includes(gui):
    result = ""
    if gui is not None:
        result += GUI_INCLUDE_STR
    return result


def statemachine_includes(statemachine, visual):
    result = ""
    if statemachine is not None:
        result += "#include <QStateMachine>\n"
        result += "#include <QState>\n"
        if visual:
            result += "#include \"statemachinewidget/qstateMachineWrapper.h\"\n"
    return result


def interfaces_includes(component, pool):
    result = ""
    for iface in sorted(list(set(component.recursiveImports + component.ice_interfaces_names))):
        name = iface.split('/')[-1].split('.')[0]
        result += '#include <' + name + '.h>\n'
    if component.usingROS is True:
        result += '#include <ros/ros.h>\n'
        for iface in sorted(pool.rosImports()):
            name = iface.split('/')[-1].split('.')[0]
            result += '#include <' + name + '.h>\n'
        for iface in component.requires + component.implements:
            if type(iface) == str:
                iface_name = iface
            else:
                iface_name = iface[0]
            if not communication_is_ice(iface):
                module = pool.moduleProviding(iface_name)
                for interface in module['interfaces']:
                    if interface['name'] == iface_name:
                        for method_name in interface['methods']:
                            result += '#include <' + module['name'] + 'ROS/' + method_name + '.h>\n'
    return result


def agm_includes(component):
    result = ""
    try:
        if component.is_agm1_agent():
            result += "#include <agm.h>\n"
        if component.is_agm2_agent():
            result += "#include <AGM2.h>\n"
            result += "#include <agm2.h>\n"
    except:
        pass
    return result


def namespaces(component):
    result = ""
    for imp in sorted(list(set(component.recursiveImports + component.ice_interfaces_names))):
        name = imp.split('/')[-1].split('.')[0]
        result += "using namespace RoboComp" + name + ";\n"
    return result


def ice_proxies_map(component):
    result = ""
    if component.language.lower() == 'cpp':
        result += "typedef map <string,::IceProxy::Ice::Object*> MapPrx;\n"
    else:
        proxy_list = []
        for name in component.requires + component.publishes:
            while not isinstance(name, str):
                name = name[0]
            proxy_list.append("RoboComp{name}::{name}PrxPtr".format(name=name))
        result += "using TuplePrx = std::tuple<" + ",".join(proxy_list) + ">;\n"
    return result


AGM_BEHAVIOUR_STRUCT_STR = """
struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};
"""


def agm_behaviour_parameter_struct(component):
    result = ""
    try:
        if 'agmagent' in [x.lower() for x in component.options]:
            result += AGM_BEHAVIOUR_STRUCT_STR
    except:
        pass
    return result


PUBLISHES_CLASS_STR = """
    //class for rosPublisher
class Publisher${iface_name}
{
public:
    ${publishers_creation}
    Publisher${iface_name}(ros::NodeHandle *node)
    {
${publishers_init}
    }
    ~Publisher${iface_name}(){}
${publishers_methods}
};
"""

REQUIRES_CLASS_STR = """
<TABHERE>//class for rosServiceClient
class ServiceClient${iface_name}
{
public:
    ${service_clients_creation}
    ServiceClient${iface_name}(ros::NodeHandle *node)
    {
${service_clients_inits}
    }
    ~ServiceClient${iface_name}(){}
    ${service_clients_methods}
};   
"""


def ros_create_publish_class(iface_name, module):
    publishers = ""
    publishers_inits = ""
    publishers_methods = ""
    for interface in module['interfaces']:
        if interface['name'] == iface_name:
            for method_name in interface['methods']:
                publishers += "<TABHERE>ros::Publisher pub_" + method_name + ";\n"
                publishers_inits += ros_node_init(interface, method_name, module, publish=True)
                publishers_methods += ros_publish_method(interface, method_name, module)
    return Template(PUBLISHES_CLASS_STR).substitute(publishers_creation=publishers,
                                                    publishers_init=publishers_inits,
                                                    publishers_methods=publishers_methods)

def ros_create_require_class(iface_name, module, idsl):
    service_clients = ""
    service_clients_inits = ""
    service_clients_methods = ""
    for interface in module['interfaces']:
        if interface['name'] == iface_name:
            for method_name in interface['methods']:
                service_clients += "<TABHERE>ros::ServiceClient srv_" + method_name + ";"
                service_clients_inits += ros_node_init(interface, method_name, module)
                service_clients_methods += ros_require_method(interface, method_name, module,idsl)
    return Template(REQUIRES_CLASS_STR).substitute(publishers_creation=service_clients,
                                                    service_clients_inits=service_clients_inits,
                                                    service_clients_methods=service_clients_methods)


def ros_param_type(p, module):
    if p['type'] in ('float', 'int'):
        param_type = p['type'].capitalize() + "32"
    elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
        param_type = "std_msgs::UInt" + p['type'].split('t')[1] + "32"
    elif p['type'] in IDSLPool.getRosTypes():
        param_type = "std_msgs::" + p['type'].capitalize()
    elif '::' in p['type']:
        param_type = p['type']
    else:
        param_type = module['name'] + "ROS::" + p['type']
    return param_type


ROS_ADVERTISE_STR = "<TABHERE><TABHERE>pub_${method_name} = node->advertise<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"
ROS_SERVICE_STR =   "<TABHERE><TABHERE>srv_${method_name} = node->serviceClient<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"


def ros_node_init(interface, method_name, module, publish=False):
    result = ""
    method = interface['methods'][method_name]
    for p in method['params']:
        param_type = ros_param_type(p, module)
        if publish:
            result += Template(ROS_ADVERTISE_STR).substitute(method_name=method_name,
                                                             param_type=param_type)
        else:
            result += Template(ROS_SERVICE_STR).substitute(method_name=method_name,
                                                           param_type=module['name'] + "ROS::" + method_name)
    return result


ROS_PUBLISH_METHOD_STR = """
    void ${method_name}(std_msgs::${type} ${param_name})
    {
        pub_${method_name}.publish(${param_name});
    }

"""

def ros_publish_method(interface, method_name, module):
    result = ""
    method = interface['methods'][method_name]
    for p in method['params']:
        param_type = ros_param_type(p, module)
        result += Template(ROS_PUBLISH_METHOD_STR).substitute(method_name=method_name,
                                                              param_type=param_type,
                                                              param_name=p['name'])
    return result

# TODO: reestructurate
def ros_require_method(interface, method_name, module, idsl):
    result = ""
    method = interface['methods'][method_name]
    methodDef = "<TABHERE>bool " + method_name + "("
    methodContent = "<TABHERE>{\n<TABHERE><TABHERE>" + module['name'] + "ROS::" + method_name + " srv;\n"
    firstParam = True
    for p in method['params']:
        for im in idsl['module']['contents']:
            # obtener todos los campos del struct y hacer la asignacion
            if firstParam:
                if im['name'] == p['type'] and im['type'] == 'struct':
                    for campos in im['structIdentifiers']:
                        methodContent += "<TABHERE><TABHERE>srv.request." + p['name'] + "." + \
                                         campos['identifier'] + " = " + p['name'] + "." + campos[
                                             'identifier'] + ";\n"
            else:
                if im['name'] == p['type'] and im['type'] == 'struct':
                    for campos in im['structIdentifiers']:
                        methodContent += "<TABHERE><TABHERE><TABHERE>" + p['name'] + "." + campos[
                            'identifier'] + " = srv.response." + p['name'] + "." + campos[
                                             'identifier'] + ";\n"
        if firstParam:
            if p['type'] in ('float', 'int'):
                methodDef += "std_msgs::" + p['type'].capitalize() + "32 " + p['name'] + ", "
                methodContent += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                    'name'] + ".data;\n"
            elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                methodDef += "std_msgs::UInt" + p['type'].split('t')[1] + " " + p['name'] + ", "
                methodContent += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                    'name'] + ".data;\n"
            elif p['type'] in IDSLPool.getRosTypes():
                methodDef += "std_msgs::" + p['type'].capitalize() + " " + p['name'] + ", "
                methodContent += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                    'name'] + ".data;\n"
            elif '::' in p['type']:
                methodDef += p['type'].replace("::", "ROS::") + " " + p['name'] + ", "
                methodContent += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                    'name'] + ";\n"
            else:
                methodDef += module['name'] + "ROS::" + p['type'] + " " + p['name'] + ", "
            methodContent += "<TABHERE><TABHERE>if(srv_" + method_name + ".call(srv))\n<TABHERE><TABHERE>{\n"
            firstParam = False
        else:
            firstParam = True
            if p['type'] in ('float', 'int'):
                methodDef += "std_msgs::" + p['type'].capitalize() + "32 &" + p['name'] + ") "
                methodContent += "<TABHERE><TABHERE><TABHERE>" + p[
                    'name'] + ".data = srv.response." + p['name'] + ";\n"
            elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                methodDef += "std_msgs::UInt" + p['type'].split('t')[1] + " &" + p['name'] + ") "
                methodContent += "<TABHERE><TABHERE><TABHERE>" + p[
                    'name'] + ".data = srv.response." + p['name'] + ";\n"
            elif p['type'] in IDSLPool.getRosTypes():
                methodDef += "std_msgs::" + p['type'].capitalize() + " &" + p['name'] + ") "
                methodContent += "<TABHERE><TABHERE><TABHERE>" + p[
                    'name'] + ".data = srv.response." + p['name'] + ";\n"
            elif '::' in p['type']:
                methodDef += p['type'].replace("::", "ROS::") + " " & +p['name'] + ") "
                methodContent += "<TABHERE><TABHERE><TABHERE>" + p['name'] + " = srv.response." + p[
                    'name'] + ";\n"
            else:
                methodDef += module['name'] + "ROS::" + p['type'] + " &" + p['name'] + ") "
            methodContent += "<TABHERE><TABHERE><TABHERE>return true;\n<TABHERE><TABHERE>}\n<TABHERE><TABHERE>return false;"
    result += methodDef+'\n'
    result += methodContent+'\n'
    result += "<TABHERE>}\n"
    return result


def ros_publishes_classes(component, pool):
    # Creating ros publisher classes
    for iface in component.publishes:
        iface_name = iface
        while not isinstance(iface_name, str):
            iface_name = iface_name[0]
        module = pool.moduleProviding(iface_name)
        if module is None:
            raise ValueError('\nCan\'t find module providing %s \n' % iface_name)
        if not communication_is_ice(iface):
            ros_create_publish_class(iface_name, module)

def ros_requires_classes(component, pool):
    for iface in component.requires:
        iface_name = iface
        while type(iface_name) != type(''):
            iface_name = iface_name[0]
        module = pool.moduleProviding(iface_name)
        theIdsl = pool.IDSLsModule(module)
        idsl = DSLFactory.from_file(theIdsl)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s\n' % iface_name)
        if not communication_is_ice(iface):
            ros_create_require_class(iface_name, module, idsl)

def agm_methods(component):
    result = ""
    try:
        if 'agmagent' in [x.lower() for x in component.options]:
            result += "bool activate(const BehaviorParameters& parameters);\n"
            result += "bool deactivate();\n"
            result += "bool isActive() { return active; }\n"
    except:
        pass
    return result

def create_proxies(component):
    result = ""
    for iface, num in get_name_number(component.requires):
        if communication_is_ice(iface):
            name = iface[0]
            if component.language.lower() == "cpp":
                result += name + 'Prx ' + name.lower() + num + '_proxy;\n'
            else:
                result += name + 'PrxPtr ' + name.lower() + num + '_proxy;\n'

    for iface, num in get_name_number(component.publishes):
        if communication_is_ice(iface):
            name = iface[0]
            if component.language.lower() == "cpp":
                result += name + 'Prx ' + name.lower() + num + '_pubproxy;\n'
            else:
                result += name + 'PrxPtr ' + name.lower() + num + '_pubproxy;\n'
    return result

def implements(component, pool):
    result = ""
    for impa in component.implements:
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
                    if communication_is_ice(impa):
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
                        result += "virtual " + method['return'] + ' ' + interface['name'] + "_" + method['name'] + "(" + paramStrA + ") = 0;\n"
                    else:
                        paramStrA = module['name'] + "ROS::" + method['name'] + "::Request &req, " + module[
                            'name'] + "ROS::" + method['name'] + "::Response &res"
                        if imp in component.iceInterfaces:
                            result += "virtual bool ROS" + method['name'] + "(" + paramStrA + ") = 0;\n"
                        else:
                            result += "virtual bool " + method['name'] + "(" + paramStrA + ") = 0;\n"
    return result

def subscribes(component, pool):
    result = ""
    for impa in component.subscribesTo:
        if type(impa) == str:
            imp = impa
        else:
            imp = impa[0]
        module = pool.moduleProviding(imp)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s \n' % imp)
        for interface in module['interfaces']:
            if interface['name'] == imp:
                for mname in interface['methods']:
                    method = interface['methods'][mname]
                    paramStrA = ''
                    if communication_is_ice(impa):
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
                            paramStrA += delim + const + p['type'] + " " + ampersand + p['name']
                        result += "virtual " + method['return'] + " " + interface['name'] + "_" + method['name'] + "(" + paramStrA + ") = 0;\n"
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
                            elif p['type'] in IDSLPool.getRosTypes():
                                p['type'] = "std_msgs::" + p['type'].capitalize()
                            elif not '::' in p['type']:
                                p['type'] = module['name'] + "ROS::" + p['type']
                            # STR
                            paramStrA += delim + p['type'] + " " + p['name']
                        if imp in component.iceInterfaces:
                            result += "virtual void ROS" + method['name'] + "(" + paramStrA + ") = 0;\n"
                        else:
                            result += "virtual void " + method['name'] + "(" + paramStrA + ") = 0;\n"
    return result

def statemachine_creation(sm, visual):
    result = ""
    if sm is not None:
        codQState = ""
        codQStateMachine = ""
        lsstates = ""
        if not visual:
            codQStateMachine = "QStateMachine " + sm['machine']['name'] + ";\n"
        else:
            codQStateMachine = "QStateMachineWrapper " + sm['machine']['name'] + ";\n"
        if sm['machine']['contents']['states'] is not None:
            for state in sm['machine']['contents']['states']:
                aux = "QState *" + state + "State;\n"
                lsstates += state + ","
                if sm['substates'] is not None:
                    for substates in sm['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] is "parallel":
                                aux = "QState *" + state + "State;\n"
                                break
                codQState += aux
        if sm['machine']['contents']['initialstate'] is not None:
            state = sm['machine']['contents']['initialstate']
            aux = "QState *" + state + "State;\n"
            lsstates += state + ","
            if sm['substates'] is not None:
                for substates in sm['substates']:
                    if state == substates['parent']:
                        if substates['parallel'] is "parallel":
                            aux = "QState *" + state + "State;\n"
                            break
            codQState += aux

        if sm['machine']['contents']['finalstate'] is not None:
            state = sm['machine']['contents']['finalstate']
            codQState += "QFinalState *" + state + "State;\n"
            lsstates += state + ","

        if sm['substates'] is not None:
            for substates in sm['substates']:
                if substates['contents']['states'] is not None:
                    for state in substates['contents']['states']:
                        aux = "QState *" + state + "State;\n"
                        lsstates += state + ","
                        for sub in sm['substates']:
                            if state == sub['parent']:
                                if sub['parallel'] is "parallel":
                                    aux = "QState *" + state + "State;\n"
                                    break
                        codQState += aux
                if substates['contents']['initialstate'] is not None:
                    aux = "QState *" + substates['contents']['initialstate'] + "State;\n"
                    lsstates += state + ","
                    for sub in sm['substates']:
                        if state == sub['parent']:
                            if sub['parallel'] is "parallel":
                                aux = "QState *" + state + "State;\n"
                                break
                    codQState += aux
                if substates['contents']['finalstate'] is not None:
                    codQState += "QFinalState *" + substates['contents']['finalstate'] + "State;\n"
                    lsstates += state + ","

        result += "//State Machine\n"
        result += codQStateMachine+"\n"
        result += codQState+"\n"
        result += "//-------------------------\n"
    return result

def ros_subscribers_creation(component, pool):
    result = ""
    for imp in component.subscribesTo:
        nname = imp
        while type(nname) != type(''):
            nname = nname[0]
        module = pool.moduleProviding(nname)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s\n' % nname)
        if not communication_is_ice(imp):
            for interface in module['interfaces']:
                if interface['name'] == nname:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        result += "ros::Subscriber " + nname + "_" + mname + ";\n"
    return result

def ros_implements_creation(component, pool):
    result = ""
    for imp in component.implements:
        nname = imp
        while type(nname) != type(''):
            nname = nname[0]
        module = pool.moduleProviding(nname)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s \n' % nname)
        if not communication_is_ice(imp):
            for interface in module['interfaces']:
                if interface['name'] == nname:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        result += "ros::ServiceServer " + nname + "_" + mname + ";\n"
    return result

def ros_publishes_creation(component):
    result = ""
    if 'publishes' in component:
        for publish in component.publishes:
            pubs = publish
            while type(pubs) != type(''):
                pubs = pubs[0]
            if not communication_is_ice(publish):
                if pubs in component.iceInterfaces:
                    result += "Publisher" + pubs + " *" + pubs.lower() + "_rosproxy;\n"
                else:
                    result += "Publisher" + pubs + " *" + pubs.lower() + "_proxy;\n"
    return result

def ros_requires_creation(component):
    result = ""
    if 'requires' in component:
        for require in component.requires:
            req = require
            while type(req) != type(''):
                req = req[0]
            if not communication_is_ice(require):
                if req in component.iceInterfaces:
                    result += "ServiceClient" + req + " *" + req.lower() + "_rosproxy;\n"
                else:
                    result += "ServiceClient" + req + " *" + req.lower() + "_proxy;\n"
    return result

AGM_ATTRIBUTES_STR = """
<TABHERE>bool active;
<TABHERE>AGMModel::SPtr worldModel;
<TABHERE>BehaviorParameters p;
<TABHERE>ParameterMap params;
<TABHERE>int iter;
<TABHERE>bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
<TABHERE>RoboCompPlanning::Action createAction(std::string s);
"""

def agm_attributes_creation(component):
    result = ""
    try:
        if 'agmagent' in [x.lower() for x in component.options]:
            result += AGM_ATTRIBUTES_STR
    except:
        pass
    return result

# TODO: Refactor for submachines
def statemachine_slots(statemachine):
    result = ""
    if statemachine is not None:
        sm_virtual_methods = ""
        if statemachine['machine']['contents']['states'] is not None:
            for state in statemachine['machine']['contents']['states']:
                sm_virtual_methods += "virtual void sm_" + state + "() = 0;\n"
        if statemachine['machine']['contents']['initialstate'] is not None:
            sm_virtual_methods += "virtual void sm_" + statemachine['machine']['contents']['initialstate'] + "() = 0;\n"
        if statemachine['machine']['contents']['finalstate'] is not None:
            sm_virtual_methods += "virtual void sm_" + statemachine['machine']['contents']['finalstate'] + "() = 0;\n"
        if statemachine['substates'] is not None:
            for substates in statemachine['substates']:
                if substates['contents']['states'] is not None:
                    for state in substates['contents']['states']:
                        sm_virtual_methods += "virtual void sm_" + state + "() = 0;\n"
                if substates['contents']['initialstate'] is not None:
                    sm_virtual_methods += "virtual void sm_" + substates['contents'][
                        'initialstate'] + "() = 0;\n"
                if substates['contents']['finalstate'] is not None:
                    sm_virtual_methods += "virtual void sm_" + substates['contents'][
                        'finalstate'] + "() = 0;\n"
        result += "//Slots funtion State Machine\n"
        result += sm_virtual_methods + '\n'
        result += "//-------------------------\n"
    return result

def statemachine_signals(statemachine):
    result = ""
    if statemachine is not None:
        codsignals = ""
        if statemachine['machine']['contents']['transitions'] is not None:
            for transi in statemachine['machine']['contents']['transitions']:
                for dest in transi['dests']:
                    codsignals += "void t_" + transi['src'] + "_to_" + dest + "();\n"
        if statemachine['substates'] is not None:
            for substates in statemachine['substates']:
                if substates['contents']['transitions'] is not None:
                    for transi in substates['contents']['transitions']:
                        for dest in transi['dests']:
                            codsignals += "void t_" + transi['src'] + "_to_" + dest + "();\n"
        result += "//Signals for State Machine\n"
        result += codsignals + '\n'
        result += "//-------------------------\n"
    return result
