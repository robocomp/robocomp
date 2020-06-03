from string import Template

from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand
from . import function_utils as utils

INTERFACE_METHOD_STR = """
${ret} ${interface_name}I::${method_name}(${input_params})
{
	${to_return}worker->${interface_name}_${method_name}(${param_str});
}
"""

CPP_TYPES = ['int', 'float', 'bool', 'void']

def interface_methods_definition(component, module, interface_name):
    result = ""
    pool = component.idsl_pool
    for interface in module['interfaces']:
        if interface['name'] == interface_name:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                ret = utils.get_type_string(method['return'], module['name'])
                name = method['name']

                param_str = utils.get_parameters_string(method, module['name'], component.language, True)
                if param_str:
                    param_str = f"{param_str}, const Ice::Current&"
                else:
                    param_str = "const Ice::Current&"
                result += ret + ' ' + name + '(' + param_str + ');\n'
    return result


def interface_methods_creation(component, interface_name):
    result = ""
    pool = component.idsl_pool
    module = pool.module_providing_interface(interface_name)
    if module is None:
        return result
    for interface in module['interfaces']:
        if interface['name'] == interface_name:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                ret = utils.get_type_string(method['return'], module['name'])
                name = method['name']

                param_str_a = utils.get_parameters_string(method,module['name'], component.language, True)
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
