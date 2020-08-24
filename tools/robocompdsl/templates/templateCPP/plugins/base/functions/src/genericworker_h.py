import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

GUI_INCLUDE_STR = """
#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
"""


class genericworker_h(TemplateDict):

    def __init__(self, component):
        super(genericworker_h, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['gui_includes'] = self.gui_includes()
        self['statemachine_includes'] = self.statemachine_includes()
        self['interfaces_includes'] = self.interfaces_includes()
        # self['namespaces'] = self.namespaces()
        self['ice_proxies_map'] = self.ice_proxies_map()
        self['inherited_object'] = self.inherited_object()
        self['constructor_proxies'] = self.constructor_proxies()
        self['create_proxies'] = self.create_proxies()
        self['implements'] = self.implements()
        self['subscribes'] = self.subscribes()
        self['statemachine_creation'] = self.statemachine_creation()
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
                            pass
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
                            pass
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
