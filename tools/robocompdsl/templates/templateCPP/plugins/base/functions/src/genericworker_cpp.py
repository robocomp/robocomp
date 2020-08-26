import datetime
from string import Template
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice
from templates.common.templatedict import TemplateDict


CPP_TYPES = ['int', 'float', 'bool', 'void']

class genericworker_cpp(TemplateDict):
    def __init__(self, component):
        super(genericworker_cpp, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['constructor_proxies'] = self.constructor_proxies()
        self['inherited_constructor'] = self.inherited_constructor()
        self['require_and_publish_proxies_creation'] = self.require_and_publish_proxies_creation()
        self['compute_connect'] = self.compute_connect()

    def require_and_publish_proxies_creation(self):
        result = ""
        cont = 0
        for interface, num in get_name_number(self.component.requires):
            if communication_is_ice(interface):
                name = interface.name
                if self.component.language.lower() == 'cpp':
                    prx_type = name
                    if prx_type not in CPP_TYPES and '::' not in prx_type:
                        module = self.component.idsl_pool.module_providing_interface(name)
                        prx_type = f"{module['name']}::{prx_type}"
                    result += name.lower() + num + "_proxy = (*(" + prx_type + "Prx*)mprx[\"" + name + "Proxy" + num + "\"]);\n"
                else:
                    result += name.lower() + num + "_proxy = std::get<" + str(cont) + ">(tprx);\n"
            cont = cont + 1

        for interface, num in get_name_number(self.component.publishes):
            if communication_is_ice(interface):
                name = interface.name
                prx_type = name
                if prx_type not in CPP_TYPES and '::' not in prx_type:
                    module = self.component.idsl_pool.module_providing_interface(name)
                    prx_type = f"{module['name']}::{prx_type}"
                if self.component.language.lower() == 'cpp':
                    result += name.lower() + num + "_pubproxy = (*(" + prx_type + "Prx*)mprx[\"" + name + "Pub" + num + "\"]);\n"
                else:
                    result += name.lower() + num + "_pubproxy = std::get<" + str(cont) + ">(tprx);\n"
            cont = cont + 1
        return result

    def constructor_proxies(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "MapPrx& mprx"
        else:
            result += "TuplePrx tprx"
        return result

    def inherited_constructor(self):
        if self.component.gui:
            return "Ui_guiDlg()"
        else:
            return "QObject()"

    def compute_connect(self):
        result = ""
        if self.component.statemachine_path is None:
            result += "connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));\n"
        return result
