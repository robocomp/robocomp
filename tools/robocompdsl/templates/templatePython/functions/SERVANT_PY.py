import datetime
from string import Template

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
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