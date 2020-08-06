import datetime

from templates.common.templatedict import TemplateDict
import templates.templateCPP.plugins.base.functions.function_utils as utils


INTERFACE_METHOD_STR = """
${ret} ${interface_name}I::${method_name}(${input_params})
{
	${to_return}worker->${interface_name}_${method_name}(${param_str});
}
"""

class SERVANT_H(TemplateDict):
    def __init__(self, component, interface_name):
        super().__init__()
        self.component = component
        module = self.component.idsl_pool.module_providing_interface(interface_name)
        self['year'] = str(datetime.date.today().year)
        self['interface_name'] = interface_name
        self['interface_name_upper'] = interface_name.upper()
        self['filename_without_extension'] = module['filename'].split('/')[-1].split('.')[0]
        self['module_name'] = module['name']
        self['interface_methods_definition'] = self.interface_methods_definition(module,
                                                                                 interface_name)

    def interface_methods_definition(self, module, interface_name):
        result = ""
        for interface in module['interfaces']:
            if interface['name'] == interface_name:
                for mname in interface['methods']:
                    method = interface['methods'][mname]

                    ret = utils.get_type_string(method['return'], module['name'])
                    name = method['name']

                    param_str = utils.get_parameters_string(method, module['name'], self.component.language)
                    if param_str:
                        param_str = f"{param_str}, const Ice::Current&"
                    else:
                        param_str = "const Ice::Current&"
                    result += ret + ' ' + name + '(' + param_str + ');\n'
        return result
