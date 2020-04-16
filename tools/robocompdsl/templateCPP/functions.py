from dsl_parsers.parsing_utils import communication_is_ice


def specificworker_implements_method_definition(pool, component):
    result = ""
    if 'implements' in component:
        for impa in component['implements']:
            if type(impa) == str:
                imp = impa
            else:
                imp = impa[0]
            module = pool.moduleProviding(imp)
            for interface in module['interfaces']:
                if interface['name'] == imp:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        paramStrA = ''
                        if communication_is_ice(impa):
                            for p in method['params']:
                                # delim
                                if paramStrA == '':
                                    delim = ''
                                else:
                                    delim = ', '
                                # decorator
                                ampersand = '&'
                                if p['decorator'] == 'out':
                                    const = ''
                                else:
                                    if component['language'].lower() == "cpp":
                                        const = 'const '
                                    else:
                                        const = ''
                                        ampersand = ''
                                    if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                                        ampersand = ''
                                # STR
                                paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
                            result += "<TABHERE>" + method['return'] + ' ' + interface['name'] + "_" + method[
                                'name'] + '(' + paramStrA + ");\n"
                        else:
                            paramStrA = module['name'] + "ROS::" + method['name'] + "::Request &req, " + module[
                                'name'] + "ROS::" + method['name'] + "::Response &res"
                            if imp in component['iceInterfaces']:
                                result +=  "<TABHERE>bool ROS" + method['name'] + '(' + paramStrA + ");\n"
                            else:
                                result +=  "<TABHERE>bool " + method['name'] + '(' + paramStrA + ");\n"
    return result