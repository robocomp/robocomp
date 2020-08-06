import datetime
from string import Template

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
"""


INTERFACE_METHOD_STR = """
def ${method_name}(self, ${params_str_a}c):
    return self.worker.${interface_name}_${method_name}(${params_str_b})
"""


class TemplateDict(dict):
    def __init__(self, component, interface_name):
        super().__init__()
        self.component = component
        module = self.component.idsl_pool.module_providing_interface(interface_name)
        if module is None:
            raise ValueError(' Can\'t locate %s' % interface_name)
        self['year'] = str(datetime.date.today().year)
        self['slice_loading'] = self.slice_loading(interface_name)
        self['interface_methods'] = self.interface_methods(module, interface_name)
        self['module_name'] = module['name']
        self['iface_name'] = interface_name

    @staticmethod
    def slice_loading(interface_name):
        result = Template(SLICE_LOAD_STR).substitute(interface_name=interface_name)
        return result

    @staticmethod
    def interface_methods(module, interface_name):
        result = ""
        for interface in module['interfaces']:
            if interface['name'] == interface_name:
                for mname in interface['methods']:
                    method = interface['methods'][mname]

                    name = method['name']

                    param_str_a = ''
                    for p in method['params']:
                        if p['decorator'] != 'out':
                            param_str_a += p['name'] + ', '
                    param_str_b = ''
                    for p in method['params']:
                        if p['decorator'] != 'out':
                            if param_str_b == '':
                                delim = ''
                            else:
                                delim = ', '
                            param_str_b += delim + p['name']

                    result += Template(INTERFACE_METHOD_STR).substitute(method_name=name,
                                                                        params_str_a=param_str_a,
                                                                        interface_name=interface_name,
                                                                        params_str_b=param_str_b)
        return result
