import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict


class genericworker_h(TemplateDict):

    def __init__(self, component):
        super(genericworker_h, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['interfaces_includes'] = self.interfaces_includes()
        # self['namespaces'] = self.namespaces()
        self['ice_proxies_map'] = self.ice_proxies_map()
        self['inherited_object'] = self.inherited_object()
        self['constructor_proxies'] = self.constructor_proxies()
        self['create_proxies'] = self.create_proxies()
        self['implements'] = self.implements()
        self['subscribes'] = self.subscribes()
        self['virtual_compute'] = self.virtual_compute()


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
            for name in sorted(self.component.requires) + sorted(self.component.publishes):
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
