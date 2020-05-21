import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number

GUI_INCLUDE_STR = """
#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
"""

AGM_BEHAVIOUR_STRUCT_STR = """
struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};
"""


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
//class for rosServiceClient
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

ROS_ADVERTISE_STR = "<TABHERE><TABHERE>pub_${method_name} = node->advertise<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"
ROS_SERVICE_STR =   "<TABHERE><TABHERE>srv_${method_name} = node->serviceClient<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"


ROS_PUBLISH_METHOD_STR = """
    void ${method_name}(std_msgs::${type} ${param_name})
    {
        pub_${method_name}.publish(${param_name});
    }

"""


AGM_ATTRIBUTES_STR = """
bool active;
AGMModel::SPtr worldModel;
BehaviorParameters p;
ParameterMap params;
int iter;
bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
RoboCompPlanning::Action createAction(std::string s);
"""


class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['gui_includes'] = self.gui_includes()
        self['statemachine_includes'] = self.statemachine_includes()
        self['interfaces_includes'] = self.interfaces_includes()
        self['agm_includes'] = self.agm_includes()
        self['namespaces'] = self.namespaces()
        self['ice_proxies_map'] = self.ice_proxies_map()
        self['agm_behaviour_parameter_struct'] = self.agm_behaviour_parameter_struct()
        self['ros_publishes_classes'] = self.ros_publishes_classes()
        self['ros_requires_classes'] = self.ros_requires_classes()
        self['inherited_object'] = self.inherited_object()
        self['constructor_proxies'] = self.constructor_proxies()
        self['agm_methods'] = self.agm_methods()
        self['create_proxies'] = self.create_proxies()
        self['implements'] = self.implements()
        self['subscribes'] = self.subscribes()
        self['statemachine_creation'] = self.statemachine_creation()
        self['ros_interfaces_creation'] = self.ros_interfaces_creation()
        self['agm_attributes_creation'] = self.agm_attributes_creation()
        self['statemachine_slots'] = self.statemachine_slots()
        self['virtual_compute'] = self.virtual_compute()
        self['statemachine_signals'] = self.statemachine_signals()

    def gui_includes(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_INCLUDE_STR
        return result


    def statemachine_includes(self):
        result = ""
        if self.component.statemachine is not None:
            result += "#include <QStateMachine>\n"
            result += "#include <QState>\n"
            if self.component.statemachine_visual:
                result += "#include \"statemachinewidget/qstateMachineWrapper.h\"\n"
        return result


    def interfaces_includes(self):
        result = ""
        pool = self.component.idsl_pool
        for iface in sorted(list(set(self.component.recursiveImports + self.component.ice_interfaces_names))):
            name = iface.split('/')[-1].split('.')[0]
            result += '#include <' + name + '.h>\n'
        if self.component.usingROS is True:
            result += '#include <ros/ros.h>\n'
            for iface in sorted(self.component.idsl_pool.ros_imports()):
                result += '#include <' + iface.name + '.h>\n'
            for iface in self.component.requires + self.component.implements:
                if type(iface) == str:
                    iface_name = iface
                else:
                    iface_name = iface[0]
                if not communication_is_ice(iface):
                    module = pool.module_providing_interface(iface_name)
                    for interface in module['interfaces']:
                        if interface['name'] == iface_name:
                            for method_name in interface['methods']:
                                result += '#include <' + module['name'] + 'ROS/' + method_name + '.h>\n'
        return result


    def agm_includes(self):
        result = ""
        try:
            if self.component.is_agm1_agent():
                result += "#include <agm.h>\n"
            if self.component.is_agm2_agent():
                result += "#include <AGM2.h>\n"
                result += "#include <agm2.h>\n"
        except:
            pass
        return result


    def namespaces(self):
        result = ""
        for imp in sorted(list(set(self.component.recursiveImports + self.component.ice_interfaces_names))):
            name = imp.split('/')[-1].split('.')[0]
            result += "using namespace RoboComp" + name + ";\n"
        return result


    def ice_proxies_map(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "typedef map <string,::IceProxy::Ice::Object*> MapPrx;\n"
        else:
            proxy_list = []
            for name in self.component.requires + self.component.publishes:
                while not isinstance(name, str):
                    name = name[0]
                proxy_list.append("RoboComp{name}::{name}PrxPtr".format(name=name))
            result += "using TuplePrx = std::tuple<" + ",".join(proxy_list) + ">;\n"
        return result

    def agm_behaviour_parameter_struct(self):
        result = ""
        try:
            if 'agmagent' in [x.lower() for x in self.component.options]:
                result += AGM_BEHAVIOUR_STRUCT_STR
        except:
            pass
        return result

    def ros_create_publish_class(self, iface_name, module):
        publishers = ""
        publishers_inits = ""
        publishers_methods = ""
        for interface in module['interfaces']:
            if interface['name'] == iface_name:
                for method_name in interface['methods']:
                    publishers += "<TABHERE>ros::Publisher pub_" + method_name + ";\n"
                    publishers_inits += self.ros_node_init(interface, method_name, module, publish=True)
                    publishers_methods += self.ros_publish_method(interface, method_name, module)
        return Template(PUBLISHES_CLASS_STR).substitute(publishers_creation=publishers,
                                                        publishers_init=publishers_inits,
                                                        publishers_methods=publishers_methods)

    def ros_create_require_class(self, iface_name, module, idsl):
        service_clients = ""
        service_clients_inits = ""
        service_clients_methods = ""
        for interface in module['interfaces']:
            if interface['name'] == iface_name:
                for method_name in interface['methods']:
                    service_clients += "<TABHERE>ros::ServiceClient srv_" + method_name + ";"
                    service_clients_inits += self.ros_node_init(interface, method_name, module)
                    service_clients_methods += self.ros_require_method(interface, method_name, module,idsl)
        return Template(REQUIRES_CLASS_STR).substitute(publishers_creation=service_clients,
                                                        service_clients_inits=service_clients_inits,
                                                        service_clients_methods=service_clients_methods)


    def ros_param_type(self, p, module):
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

    def ros_node_init(self, interface, method_name, module, publish=False):
        result = ""
        method = interface['methods'][method_name]
        for p in method['params']:
            param_type = self.ros_param_type(p, module)
            if publish:
                result += Template(ROS_ADVERTISE_STR).substitute(method_name=method_name,
                                                                 param_type=param_type)
            else:
                result += Template(ROS_SERVICE_STR).substitute(method_name=method_name,
                                                               param_type=module['name'] + "ROS::" + method_name)
        return result

    def ros_publish_method(self, interface, method_name, module):
        result = ""
        method = interface['methods'][method_name]
        for p in method['params']:
            param_type = self.ros_param_type(p, module)
            result += Template(ROS_PUBLISH_METHOD_STR).substitute(method_name=method_name,
                                                                  param_type=param_type,
                                                                  param_name=p['name'])
        return result

    # TODO: reestructurate
    def ros_require_method(self, interface, method_name, module, idsl):
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


    def ros_publishes_classes(self):
        result = ""
        if self.component.usingROS:
            pool = self.component.idsl_pool
            # Creating ros publisher classes
            for iface in self.component.publishes:
                iface_name = iface
                while not isinstance(iface_name, str):
                    iface_name = iface_name[0]
                module = pool.module_providing_interface(iface_name)
                if module is None:
                    raise ValueError('\nCan\'t find module providing %s \n' % iface_name)
                if not communication_is_ice(iface):
                    result += self.ros_create_publish_class(iface_name, module)
        return result

    def ros_requires_classes(self):
        result = ""
        if self.component.usingROS == True:
            pool = self.component.idsl_pool
            for iface in self.component.requires:
                iface_name = iface
                while type(iface_name) != type(''):
                    iface_name = iface_name[0]
                module = pool.module_providing_interface(iface_name)
                theIdsl = pool.IDSL_file_for_module(module)
                idsl = DSLFactory().from_file(theIdsl)
                if module == None:
                    raise ValueError('\nCan\'t find module providing %s\n' % iface_name)
                if not communication_is_ice(iface):
                    result += self.ros_create_require_class(iface_name, module, idsl)
        return result

    def agm_methods(self):
        result = ""
        try:
            if 'agmagent' in [x.lower() for x in self.component.options]:
                result += "bool activate(const BehaviorParameters& parameters);\n"
                result += "bool deactivate();\n"
                result += "bool isActive() { return active; }\n"
        except:
            pass
        return result

    def create_proxies(self):
        result = ""
        for iface, num in get_name_number(self.component.requires):
            if communication_is_ice(iface):
                name = iface[0]
                if self.component.language.lower() == "cpp":
                    result += name + 'Prx ' + name.lower() + num + '_proxy;\n'
                else:
                    result += name + 'PrxPtr ' + name.lower() + num + '_proxy;\n'

        for iface, num in get_name_number(self.component.publishes):
            if communication_is_ice(iface):
                name = iface[0]
                if self.component.language.lower() == "cpp":
                    result += name + 'Prx ' + name.lower() + num + '_pubproxy;\n'
                else:
                    result += name + 'PrxPtr ' + name.lower() + num + '_pubproxy;\n'
        return result

    def implements(self):
        result = ""
        for iface in self.component.implements:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            for interface in module['interfaces']:
                if interface['name'] == iface.name:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        paramStrA = ''
                        if communication_is_ice(iface):
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
                                    if self.component.language.lower() == "cpp":
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
                            if iface.name in self.component.iceInterfaces:
                                result += "virtual bool ROS" + method['name'] + "(" + paramStrA + ") = 0;\n"
                            else:
                                result += "virtual bool " + method['name'] + "(" + paramStrA + ") = 0;\n"
        return result

    def subscribes(self):
        result = ""
        for iface in self.component.subscribesTo:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            if module == None:
                raise ValueError('\nCan\'t find module providing %s \n' % iface.name)
            for interface in module['interfaces']:
                if interface['name'] == iface.name:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        paramStrA = ''
                        if communication_is_ice(iface):
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
                                    if self.component.language.lower() == "cpp":
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
                            if iface.name in self.component.iceInterfaces:
                                result += "virtual void ROS" + method['name'] + "(" + paramStrA + ") = 0;\n"
                            else:
                                result += "virtual void " + method['name'] + "(" + paramStrA + ") = 0;\n"
        return result

    def statemachine_creation(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            codQState = ""
            codQStateMachine = ""
            lsstates = ""
            if not self.component.statemachine_visual:
                codQStateMachine = "QStateMachine " + statemachine['machine']['name'] + ";\n"
            else:
                codQStateMachine = "QStateMachineWrapper " + statemachine['machine']['name'] + ";\n"
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    aux = "QState *" + state + "State;\n"
                    lsstates += state + ","
                    if statemachine['substates'] is not None:
                        for substates in statemachine['substates']:
                            if state == substates['parent']:
                                if substates['parallel'] is "parallel":
                                    aux = "QState *" + state + "State;\n"
                                    break
                    codQState += aux
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                aux = "QState *" + state + "State;\n"
                lsstates += state + ","
                if statemachine['substates'] is not None:
                    for substates in statemachine['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] is "parallel":
                                aux = "QState *" + state + "State;\n"
                                break
                codQState += aux

            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                codQState += "QFinalState *" + state + "State;\n"
                lsstates += state + ","

            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            aux = "QState *" + state + "State;\n"
                            lsstates += state + ","
                            for sub in statemachine['substates']:
                                if state == sub['parent']:
                                    if sub['parallel'] is "parallel":
                                        aux = "QState *" + state + "State;\n"
                                        break
                            codQState += aux
                    if substates['contents']['initialstate'] is not None:
                        aux = "QState *" + substates['contents']['initialstate'] + "State;\n"
                        lsstates += state + ","
                        for sub in statemachine['substates']:
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

    def ros_subscribers_creation(self):
        result = ""
        for iface in self.component.subscribesTo:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            if module == None:
                raise ValueError('\nCan\'t find module providing %s\n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            method = interface['methods'][mname]
                            result += "ros::Subscriber " + iface.name + "_" + mname + ";\n"
        return result

    def ros_implements_creation(self):
        result = ""
        for iface in self.component.implements:
            pool = self.component.pool
            module = pool.module_providing_interface(iface.name)
            if module == None:
                raise ValueError('\nCan\'t find module providing %s \n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            method = interface['methods'][mname]
                            result += "ros::ServiceServer " + iface.name + "_" + mname + ";\n"
        return result

    def ros_publishes_creation(self):
        result = ""
        if 'publishes' in self.component:
            for publish in self.component.publishes:
                pubs = publish
                while type(pubs) != type(''):
                    pubs = pubs[0]
                if not communication_is_ice(publish):
                    if pubs in self.component.iceInterfaces:
                        result += "Publisher" + pubs + " *" + pubs.lower() + "_rosproxy;\n"
                    else:
                        result += "Publisher" + pubs + " *" + pubs.lower() + "_proxy;\n"
        return result

    def ros_requires_creation(self):
        result = ""
        if 'requires' in self.component:
            for require in self.component.requires:
                req = require
                while type(req) != type(''):
                    req = req[0]
                if not communication_is_ice(require):
                    if req in self.component.iceInterfaces:
                        result += "ServiceClient" + req + " *" + req.lower() + "_rosproxy;\n"
                    else:
                        result += "ServiceClient" + req + " *" + req.lower() + "_proxy;\n"
        return result

    def agm_attributes_creation(self):
        result = ""
        try:
            if 'agmagent' in [x.lower() for x in self.component.options]:
                result += AGM_ATTRIBUTES_STR
        except:
            pass
        return result

    # TODO: Refactor for submachines
    def statemachine_slots(self):
        result = ""
        statemachine = self.component.statemachine
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

    def statemachine_signals(self):
        result = ""
        statemachine = self.component.statemachine
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

    def constructor_proxies(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "MapPrx& mprx"
        else:
            result += "TuplePrx tprx"
        return result


    def ros_interfaces_creation(self):
        result = ""
        if self.component.usingROS:
            result += "ros::NodeHandle node;\n"
            result += self.ros_subscribers_creation()
            result += self.ros_implements_creation()
            result += self.ros_publishes_creation()
            result += self.ros_requires_creation()
        return result

    def virtual_compute(self):
        result = ""
        statemachine = self.component.statemachine
        if (statemachine is not None and statemachine['machine']['default'] is True) or self.component.statemachine_path is None:
            result += "virtual void compute() = 0;\n"
        return result

    def inherited_object(self):
        if self.component.gui:
            return "public {gui_widget}, public Ui_guiDlg".format(gui_widget=self.component.gui.widget)
        else:
            return "public QObject"