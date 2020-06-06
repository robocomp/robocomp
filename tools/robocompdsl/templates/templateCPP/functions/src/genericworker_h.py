import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from .. import function_utils as utils

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
class Publisher${interface_name}
{
public:
    ${publishers_creation}
    Publisher${interface_name}(ros::NodeHandle *node)
    {
${publishers_init}
    }
    ~Publisher${interface_name}(){}
${publishers_methods}
};
"""

REQUIRES_CLASS_STR = """
//class for rosServiceClient
class ServiceClient${interface_name}
{
public:
    ${service_clients_creation}
    ServiceClient${interface_name}(ros::NodeHandle *node)
    {
${service_clients_inits}
    }
    ~ServiceClient${interface_name}(){}
    ${service_clients_methods}
};   
"""

ROS_ADVERTISE_STR = "<TABHERE><TABHERE>pub_${method_name} = node->advertise<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"
ROS_SERVICE_STR = "<TABHERE><TABHERE>srv_${method_name} = node->serviceClient<${param_type}>(node->resolveName(\"${method_name}\"), 1000);"


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
RoboCompAGMCommonBehavior::ParameterMap params;
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
        # self['namespaces'] = self.namespaces()
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
                    interface_name = iface
                else:
                    interface_name = iface[0]
                if not communication_is_ice(iface):
                    module = pool.module_providing_interface(interface_name)
                    for interface in module['interfaces']:
                        if interface['name'] == interface_name:
                            for method_name in interface['methods']:
                                result += '#include <' + module['name'] + 'ROS/' + method_name + '.h>\n'
        return result

    def agm_includes(self):
        result = ""
        if self.component.is_agm1_agent():
            result += "#include <agm.h>\n"
        if self.component.is_agm2_agent():
            result += "#include <AGM2.h>\n"
            result += "#include <agm2.h>\n"
        return result

    # def namespaces(self):
    #     result = ""
    #     for imp in sorted(list(set(self.component.recursiveImports + self.component.ice_interfaces_names))):
    #         name = imp.split('/')[-1].split('.')[0]
    #         result += "using namespace RoboComp" + name + ";\n"
    #     return result

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
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_BEHAVIOUR_STRUCT_STR
        return result

    def ros_create_publish_class(self, interface_name, module):
        publishers = ""
        publishers_inits = ""
        publishers_methods = ""
        for interface in module['interfaces']:
            if interface['name'] == interface_name:
                for method_name in interface['methods']:
                    publishers += "<TABHERE>ros::Publisher pub_" + method_name + ";\n"
                    publishers_inits += self.ros_node_init(interface, method_name, module, publish=True)
                    publishers_methods += self.ros_publish_method(interface, method_name, module)
        return Template(PUBLISHES_CLASS_STR).substitute(publishers_creation=publishers,
                                                        publishers_init=publishers_inits,
                                                        publishers_methods=publishers_methods)

    def ros_create_require_class(self, interface_name, module, idsl):
        service_clients = ""
        service_clients_inits = ""
        service_clients_methods = ""
        for interface in module['interfaces']:
            if interface['name'] == interface_name:
                for method_name in interface['methods']:
                    service_clients += "<TABHERE>ros::ServiceClient srv_" + method_name + ";"
                    service_clients_inits += self.ros_node_init(interface, method_name, module)
                    service_clients_methods += self.ros_require_method(interface, method_name, module, idsl)
        return Template(REQUIRES_CLASS_STR).substitute(publishers_creation=service_clients,
                                                       service_clients_inits=service_clients_inits,
                                                       service_clients_methods=service_clients_methods)

    @staticmethod
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

    # TODO: restructure
    @staticmethod
    def ros_require_method(interface, method_name, module, idsl):
        result = ""
        method = interface['methods'][method_name]
        method_def = "<TABHERE>bool " + method_name + "("
        method_content = "<TABHERE>{\n<TABHERE><TABHERE>" + module['name'] + "ROS::" + method_name + " srv;\n"
        first_param = True
        for p in method['params']:
            for im in idsl['module']['contents']:
                # obtener todos los campos del struct y hacer la asignacion
                if first_param:
                    if im['name'] == p['type'] and im['type'] == 'struct':
                        for campos in im['structIdentifiers']:
                            method_content += "<TABHERE><TABHERE>srv.request." + p['name'] + "." + \
                                             campos['identifier'] + " = " + p['name'] + "." + campos[
                                                 'identifier'] + ";\n"
                else:
                    if im['name'] == p['type'] and im['type'] == 'struct':
                        for campos in im['structIdentifiers']:
                            method_content += "<TABHERE><TABHERE><TABHERE>" + p['name'] + "." + campos[
                                'identifier'] + " = srv.response." + p['name'] + "." + campos[
                                                 'identifier'] + ";\n"
            if first_param:
                if p['type'] in ('float', 'int'):
                    method_def += "std_msgs::" + p['type'].capitalize() + "32 " + p['name'] + ", "
                    method_content += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                        'name'] + ".data;\n"
                elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                    method_def += "std_msgs::UInt" + p['type'].split('t')[1] + " " + p['name'] + ", "
                    method_content += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                        'name'] + ".data;\n"
                elif p['type'] in IDSLPool.getRosTypes():
                    method_def += "std_msgs::" + p['type'].capitalize() + " " + p['name'] + ", "
                    method_content += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                        'name'] + ".data;\n"
                elif '::' in p['type']:
                    method_def += p['type'].replace("::", "ROS::") + " " + p['name'] + ", "
                    method_content += "<TABHERE><TABHERE>srv.request." + p['name'] + " = " + p[
                        'name'] + ";\n"
                else:
                    method_def += module['name'] + "ROS::" + p['type'] + " " + p['name'] + ", "
                method_content += "<TABHERE><TABHERE>if(srv_" + method_name + ".call(srv))\n<TABHERE><TABHERE>{\n"
                first_param = False
            else:
                first_param = True
                if p['type'] in ('float', 'int'):
                    method_def += "std_msgs::" + p['type'].capitalize() + "32 &" + p['name'] + ") "
                    method_content += "<TABHERE><TABHERE><TABHERE>" + p[
                        'name'] + ".data = srv.response." + p['name'] + ";\n"
                elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                    method_def += "std_msgs::UInt" + p['type'].split('t')[1] + " &" + p['name'] + ") "
                    method_content += "<TABHERE><TABHERE><TABHERE>" + p[
                        'name'] + ".data = srv.response." + p['name'] + ";\n"
                elif p['type'] in IDSLPool.getRosTypes():
                    method_def += "std_msgs::" + p['type'].capitalize() + " &" + p['name'] + ") "
                    method_content += "<TABHERE><TABHERE><TABHERE>" + p[
                        'name'] + ".data = srv.response." + p['name'] + ";\n"
                elif '::' in p['type']:
                    method_def += p['type'].replace("::", "ROS::") + " " & +p['name'] + ") "
                    method_content += "<TABHERE><TABHERE><TABHERE>" + p['name'] + " = srv.response." + p[
                        'name'] + ";\n"
                else:
                    method_def += module['name'] + "ROS::" + p['type'] + " &" + p['name'] + ") "
                method_content += "<TABHERE><TABHERE><TABHERE>return true;\n<TABHERE><TABHERE>}\n<TABHERE><TABHERE>return false;"
        result += method_def+'\n'
        result += method_content+'\n'
        result += "<TABHERE>}\n"
        return result

    def ros_publishes_classes(self):
        result = ""
        if self.component.usingROS:
            pool = self.component.idsl_pool
            # Creating ros publisher classes
            for interface in self.component.publishes:
                module = pool.module_providing_interface(interface.name)
                if module is None:
                    raise ValueError('\nCan\'t find module providing %s \n' % interface.name)
                if not communication_is_ice(interface):
                    result += self.ros_create_publish_class(interface.name, module)
        return result

    def ros_requires_classes(self):
        result = ""
        if self.component.usingROS:
            pool = self.component.idsl_pool
            for interface in self.component.requires:
                module = pool.module_providing_interface(interface.name)
                idsl_path = pool.IDSL_file_for_module(module)
                idsl = DSLFactory().from_file(idsl_path)
                if module is None:
                    raise ValueError('\nCan\'t find module providing %s\n' % interface.name)
                if not communication_is_ice(interface):
                    result += self.ros_create_require_class(interface.name, module, idsl)
        return result

    def agm_methods(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += "bool activate(const BehaviorParameters& parameters);\n"
            result += "bool deactivate();\n"
            result += "bool isActive() { return active; }\n"
        return result

    def create_proxies(self):
        result = ""
        for iface, num in get_name_number(self.component.requires):
            if communication_is_ice(iface):
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                proxy_type = iface.name
                if module is not None:
                    proxy_type = f"{module['name']}::{iface.name}"
                if self.component.language.lower() == "cpp":
                    result += f"{proxy_type}Prx {iface.name.lower()}{num}_proxy;\n"
                else:
                    result += f"{proxy_type}PrxPtr {iface.name.lower()}{num}_proxy;\n"

        for iface, num in get_name_number(self.component.publishes):
            if communication_is_ice(iface):
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                proxy_type = iface.name
                if module is not None:
                    proxy_type = f"{module['name']}::{iface.name}"
                if self.component.language.lower() == "cpp":
                    result += f"{proxy_type}Prx {iface.name.lower()}{num}_pubproxy;\n"
                else:
                    result += f"{proxy_type}PrxPtr {iface.name.lower()}{num}_pubproxy;\n"
        return result

    #TODO: check if it can be mixed with the subscribes methodd. Are too similar.
    def implements(self):
        result = ""
        for iface in self.component.implements:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            for interface in module['interfaces']:
                if interface['name'] == iface.name:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        if communication_is_ice(iface):
                            param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                            return_type = utils.get_type_string(method['return'], module['name'])
                            result += f"virtual {return_type} {interface['name']}_{method['name']}({param_str_a}) = 0;\n"
                        else:
                            param_str_a = module['name'] + "ROS::" + method['name'] + "::Request &req, " + module[
                                'name'] + "ROS::" + method['name'] + "::Response &res"
                            if iface.name in self.component.iceInterfaces:
                                result += "virtual bool ROS" + method['name'] + "(" + param_str_a + ") = 0;\n"
                            else:
                                result += "virtual bool " + method['name'] + "(" + param_str_a + ") = 0;\n"
        return result

    def subscribes(self):
        result = ""
        for iface in self.component.subscribesTo:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            if module is None:
                raise ValueError('\nCan\'t find module providing %s \n' % iface.name)
            for interface in module['interfaces']:
                if interface['name'] == iface.name:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        if communication_is_ice(iface):
                            param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                            return_type = utils.get_type_string(method['return'], module['name'])
                            result += f"virtual {return_type} {interface['name']}_{method['name']} ({param_str_a}) = 0;\n"
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
                                elif p['type'] in IDSLPool.getRosTypes():
                                    p['type'] = "std_msgs::" + p['type'].capitalize()
                                elif '::' not in p['type']:
                                    p['type'] = module['name'] + "ROS::" + p['type']
                                # STR
                                param_str_a += delim + p['type'] + " " + p['name']
                            if iface.name in self.component.iceInterfaces:
                                result += "virtual void ROS" + method['name'] + "(" + param_str_a + ") = 0;\n"
                            else:
                                result += "virtual void " + method['name'] + "(" + param_str_a + ") = 0;\n"
        return result

    def statemachine_creation(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            code_qstates = ""
            lsstates = ""
            if not self.component.statemachine_visual:
                cod_qstate_machine = "QStateMachine " + statemachine['machine']['name'] + ";\n"
            else:
                cod_qstate_machine = "QStateMachineWrapper " + statemachine['machine']['name'] + ";\n"
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
                    code_qstates += aux
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
                code_qstates += aux

            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                code_qstates += "QFinalState *" + state + "State;\n"
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
                            code_qstates += aux
                    if substates['contents']['initialstate'] is not None:
                        aux = "QState *" + substates['contents']['initialstate'] + "State;\n"
                        lsstates += state + ","
                        for sub in statemachine['substates']:
                            if state == sub['parent']:
                                if sub['parallel'] is "parallel":
                                    aux = "QState *" + state + "State;\n"
                                    break
                        code_qstates += aux
                    if substates['contents']['finalstate'] is not None:
                        code_qstates += "QFinalState *" + substates['contents']['finalstate'] + "State;\n"
                        lsstates += state + ","

            result += "//State Machine\n"
            result += cod_qstate_machine+"\n"
            result += code_qstates+"\n"
            result += "//-------------------------\n"
        return result

    def ros_subscribers_creation(self):
        result = ""
        for iface in self.component.subscribesTo:
            pool = self.component.idsl_pool
            module = pool.module_providing_interface(iface.name)
            if module is None:
                raise ValueError('\nCan\'t find module providing %s\n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            result += "ros::Subscriber " + iface.name + "_" + mname + ";\n"
        return result

    def ros_implements_creation(self):
        result = ""
        for iface in self.component.implements:
            pool = self.component.pool
            module = pool.module_providing_interface(iface.name)
            if module is None:
                raise ValueError('\nCan\'t find module providing %s \n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            result += "ros::ServiceServer " + iface.name + "_" + mname + ";\n"
        return result

    def ros_publishes_creation(self):
        result = ""
        if 'publishes' in self.component:
            for publish in self.component.publishes:
                if not communication_is_ice(publish):
                    if publish.name in self.component.iceInterfaces:
                        result += "Publisher" + publish.name + " *" + publish.name.lower() + "_rosproxy;\n"
                    else:
                        result += "Publisher" + publish.name + " *" + publish.name.lower() + "_proxy;\n"
        return result

    def ros_requires_creation(self):
        result = ""
        if 'requires' in self.component:
            for require in self.component.requires:
                if not communication_is_ice(require):
                    if require.name in self.component.iceInterfaces:
                        result += "ServiceClient" + require.name + " *" + require.name.lower() + "_rosproxy;\n"
                    else:
                        result += "ServiceClient" + require.name + " *" + require.name.lower() + "_proxy;\n"
        return result

    def agm_attributes_creation(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_ATTRIBUTES_STR
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
