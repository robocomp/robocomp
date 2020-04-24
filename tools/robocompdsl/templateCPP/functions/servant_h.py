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
