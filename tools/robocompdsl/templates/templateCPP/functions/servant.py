from string import Template

from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand


def interface_methods_definition(component, module, modulePool, theInterface):
    result = ""
    for interface in module['interfaces']:
        if interface['name'] == theInterface:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                ret = method['return']
                name = method['name']

                paramStr = ''
                for p in method['params']:

                    const, ampersand = decorator_and_type_to_const_ampersand(p['decorator'], p['type'], modulePool,
                                                                             component.language.lower() == "cpp11")
                    if p['type'] == 'long':
                        paramStr += const + 'Ice::Long' + ' ' + ampersand + p['name'] + ', '
                    else:
                        paramStr += const + p['type'] + ' ' + ampersand + p['name'] + ', '

                result += ret + ' ' + name + '(' + paramStr + 'const Ice::Current&);\n'
    return result

INTERFACE_METHOD_STR = """
${ret} ${interface_name}I::${method_name}(${input_params}const Ice::Current&)
{
	${to_return}worker->${interface_name}_${method_name}(${param_str});
}
"""

def interface_methods_creation(component, modulePool, the_interface):
    result = ""
    module = modulePool.moduleProviding(the_interface[0])
    if module is None:
        return result
    for interface in module['interfaces']:
        if interface['name'] == the_interface[0]:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                ret = method['return']
                name = method['name']

                paramStrA = ''
                for p in method['params']:
                    const, ampersand = decorator_and_type_to_const_ampersand(p['decorator'], p['type'], modulePool,
                                                                             component.language.lower() == "cpp11")
                    if p['type'] == 'long':
                        paramStrA += const + 'Ice::Long' + ' ' + ampersand + p['name'] + ', '
                    else:
                        paramStrA += const + p['type'] + ' ' + ampersand + p['name'] + ', '

                paramStrB = ''
                for p in method['params']:
                    if paramStrB == '':
                        delim = ''
                    else:
                        delim = ', '
                    paramStrB += delim + p['name']

                result += Template(INTERFACE_METHOD_STR).substitute(ret=ret,
                                                                    interface_name=interface['name'],
                                                                    method_name=name,
                                                                    input_params=paramStrA,
                                                                    to_return="return " if ret != "void" else "",
                                                                    param_str=paramStrB
                                                                    )
    return result
