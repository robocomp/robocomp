from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent, IDSLPool

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
    if component.usingROS == True:
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
        if is_agm1_agent(component):
            result += "#include <agm.h>\n"
        if is_agm2_agent(component):
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
            proxy_list.append("RoboComp{name}::{name}PrxPtr".format(name=name))
        result += "using TuplePrx = std::tuple<" + ",".join(proxy_list) + ">;\n"
    return result

AGM_BEHAVIOUR_STRUCT_STR ="""
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

PUBLISHES_CLASS_STR="""
<TABHERE>//class for rosPublisher
class Publisher${iface_name}
{
public:
<TABHERE>${publishers}
<TABHERE>Publisher${iface_name}(ros::NodeHandle *node)
<TABHERE>{
${advertisers}
<TABHERE>}
<TABHERE>~Publisher${iface_name}(){}
${methods}
};
"""

def ros_create_class(iface_name, module):
    publishers = ""
    advertisers = ""
    methods = ""
    for interface in module['interfaces']:
        if interface['name'] == iface_name:
            for mname in interface['methods']:
                publishers += "<TABHERE>ros::Publisher pub_" + mname + ";\n"
                advertisers += ros_method_advertiser(interface,mname,module)
                methods += ros_method(interface,mname,module)
    return Template(PUBLISHES_CLASS_STR).substitute(publishers=publishers, advertisers=advertisers, methods=methods)

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


ROS_ADVERTISE_STR = "<TABHERE><TABHERE>pub_${method_name} = node->advertise<${param_type}>(node->resolveName(\"${method_name\"), 1000);"

def ros_method_advertiser(interface, method_name, module):
    result = ""
    method = interface['methods'][method_name]
    for p in method['params']:
        param_type = ros_param_type(p, module)
        result += Template(ROS_ADVERTISE_STR).substitute(method_name=method_name, param_type=param_type)
    return result

ROS_METHOD_STR = """
<TABHERE>void ${method_name}(std_msgs::${type} ${param_name})
<TABHERE>{
<TABHERE><TABHERE>pub_${method_name}.publish(${param_name});
<TABHERE>}

"""

def ros_method(interface, method_name, module):
    result = ""
    method = interface['methods'][method_name]
    for p in method['params']:
        param_type = ros_param_type(p, module)
        result += Template(ROS_METHOD_STR).substitute(method_name=method_name,
                                                      param_type=param_type,
                                                      param_name=p['name'])
    return result

def ros_publishes_classes(component, pool):
    # CREANDO CLASES PARA LOS PUBLISHERS
    for iface in component.publishes:
        iface_name = iface
        while type(iface_name) != type(''):
            iface_name = iface_name[0]
        module = pool.moduleProviding(iface_name)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s \n' % iface_name)
        if not communication_is_ice(iface):
            ros_create_class(iface_name, module)
