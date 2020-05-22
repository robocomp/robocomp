from string import Template

from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand


INTERFACE_METHOD_STR = """
${ret} ${interface_name}I::${method_name}(${input_params}const Ice::Current&)
{
	${to_return}worker->${interface_name}_${method_name}(${param_str});
}
"""


def interface_methods_definition(component, module, interface_name):
    result = ""
    pool = component.idsl_pool
    for interface in module['interfaces']:
        if interface['name'] == interface_name:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                ret = method['return']
                name = method['name']

                param_str = ''
                for p in method['params']:

                    const, ampersand = decorator_and_type_to_const_ampersand(p['decorator'], p['type'], pool,
                                                                             component.language.lower() == "cpp11")
                    if p['type'] == 'long':
                        param_str += const + 'Ice::Long' + ' ' + ampersand + p['name'] + ', '
                    else:
                        param_str += const + p['type'] + ' ' + ampersand + p['name'] + ', '

                result += ret + ' ' + name + '(' + param_str + 'const Ice::Current&);\n'
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

                ret = method['return']
                name = method['name']

                param_str_a = ''
                for p in method['params']:
                    const, ampersand = decorator_and_type_to_const_ampersand(p['decorator'], p['type'], pool,
                                                                             component.language.lower() == "cpp11")
                    if p['type'] == 'long':
                        param_str_a += const + 'Ice::Long' + ' ' + ampersand + p['name'] + ', '
                    else:
                        param_str_a += const + p['type'] + ' ' + ampersand + p['name'] + ', '

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
