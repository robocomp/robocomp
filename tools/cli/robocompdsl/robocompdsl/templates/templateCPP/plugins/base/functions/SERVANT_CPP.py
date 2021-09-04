from string import Template
import datetime

from robocompdsl.templates.common.templatedict import TemplateDict
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils


INTERFACE_METHOD_STR = """
${ret} ${interface_name}I::${method_name}(${input_params})
{
	${to_return}worker->${interface_name}_${method_name}(${param_str});
}
"""

class SERVANT_H(TemplateDict):
    def __init__(self, component, interface_name):
        super(SERVANT_H, self).__init__()
        self.component = component
        module = self.component.idsl_pool.module_providing_interface(interface_name)
        self['year'] = str(datetime.date.today().year)
        self['interface_name'] = interface_name
        self['interface_name_lower'] = interface_name.lower()
        self['interface_methods_creation'] = self.interface_methods_creation(interface_name)

    def interface_methods_creation(self,  interface_name):
        result = ""
        pool = self.component.idsl_pool
        module = pool.module_providing_interface(interface_name)
        if module is None:
            return result
        for interface in module['interfaces']:
            if interface['name'] == interface_name:
                for mname in interface['methods']:
                    method = interface['methods'][mname]

                    ret = utils.get_type_string(method['return'], module['name'])
                    name = method['name']

                    param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                    if param_str_a:
                        param_str_a = f"{param_str_a}, const Ice::Current&"
                    else:
                        param_str_a = "const Ice::Current&"
                    param_str_b = ''
                    for p in method['params']:
                        if param_str_b == '':
                            delim = ''
                        else:
                            delim = ', '
                        param_str_b += delim + p['name']

                    result += Template(INTERFACE_METHOD_STR).substitute(ret=ret,
                                                                        interface_name=interface['name'],
                                                                        method_name=name,
                                                                        input_params=param_str_a,
                                                                        to_return="return " if ret != "void" else "",
                                                                        param_str=param_str_b
                                                                        )
        return result