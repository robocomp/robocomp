import datetime
from string import Template

SLICE_LOAD_STR = """
ice_${interface_name} = False
for p in icePaths:
    print(\'Trying\', p, \'to load ${interface_name}.ice\')
    if os.path.isfile(p+\'/${interface_name}.ice\'):
        print(\'Using\', p, \'to load ${interface_name}.ice\')
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+\'/\'
        wholeStr = preStr+"${interface_name}.ice"
        Ice.loadSlice(wholeStr)
        ice_${interface_name} = True
        break
if not ice_${interface_name}:
    print('Couldn\\\'t load ${interface_name}')
    sys.exit(-1)
"""

def slice_loading(interface_name):
    result = Template(SLICE_LOAD_STR).substitute(interface_name=interface_name)
    return result

INTERFACE_METHOD_STR = """
def ${method_name}(self, ${params_str_a}c):
    return self.worker.${interface_name}_${method_name}(${params_str_b})
"""

def interface_methods(module, interface_name):
    result = ""
    for interface in module['interfaces']:
        if interface['name'] == interface_name:
            for mname in interface['methods']:
                method = interface['methods'][mname]

                name = method['name']

                paramStrA = ''
                for p in method['params']:
                    if p['decorator'] != 'out':
                        paramStrA += p['name'] + ', '
                paramStrB = ''
                for p in method['params']:
                    if p['decorator'] != 'out':
                        if paramStrB == '':
                            delim = ''
                        else:
                            delim = ', '
                        paramStrB += delim + p['name']

                result += Template(INTERFACE_METHOD_STR).substitute(method_name=name,
                                                                    params_str_a=paramStrA,
                                                                    interface_name=interface_name,
                                                                    params_str_b=paramStrB)
    return result

def get_template_dict(component, interface_name):
    module = component.idsl_pool.moduleProviding(interface_name)
    if module == None:
        raise ValueError(' Can\'t locate %s' % interface_name)
    return {
        'year': str(datetime.date.today().year),
        'slice_loading': slice_loading(interface_name),
        'interface_methods': interface_methods(module, interface_name),
        'module_name': module['name'],
        'iface_name': interface_name
    }