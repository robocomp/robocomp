import os
from string import Template
from robocompdsl.templates.common.templatedict import TemplateDict
from robocompdsl.templates.common.abstracttemplatesmanager import CustomTemplate as CTemplate


ICE_EXCEPTION_STR = """\
exception ${type_name}{${type_content}};
"""

ICE_SEQUENCE_STR = """\
sequence <${type_sequence}> ${type_name};
"""

ICE_DICTIONARY_STR = """\
dictionary <${type_content}> ${type_name};
"""

ICE_ENUM_STR = """\
enum ${type_name} { ${type_content} };
"""

ICE_STRUCT_STR = """\
struct ${type_name}
{
	${attributes}
};
"""

ICE_STRUCT_ATTRIBUTE_STR = """\
${var_type} ${var_identifier}${default_value};
"""


ICE_INTERFACE_STR = """\
interface ${interface_name}
{
	${interface_methods}
};
"""

ICE_METHOD_STR = """\
${method_decorator}${method_return} ${method_name} (${params_str_a})${exception};
"""


class TEMPLATE_ICE(TemplateDict):
    def __init__(self, module):
        super(TEMPLATE_ICE, self).__init__()
        self.module = module
        self['module_name'] = module['name']
        self['module_filename'] = os.path.basename(module['filename']).split('.')[0]
        self['module_file'] = os.path.basename(module['filename'])
        self['module_name_upper'] = module['name'].upper()
        self['ice_imports'] = self.ice_imports()
        self['ice_types'] = self.ice_types()
        self['ice_interfaces'] = self.ice_interfaces()

    def ice_imports(self):
        result = ""
        if 'imports' in self.module and self.module["imports"] != '':
            for imp in self.module['imports']:
                if imp != '':
                    result += "#include <" + os.path.basename(imp).split('.')[0] + ".ice>\n"
        return result

    def ice_types(self):
        result = ""
        if 'types' in self.module:
            for next_type in self.module["types"]:
                if "exception" == next_type["type"]:
                    result += Template(ICE_EXCEPTION_STR).substitute(type_name=next_type['name'],
                                                                     type_content=next_type['content'])
                if "struct" == next_type["type"]:
                    struct = next_type
                    attributes = ""
                    for var in struct['structIdentifiers']:
                        try:
                            default_value = " =" + var['defaultValue']
                        except KeyError:
                            default_value = ""
                        attributes += Template(ICE_STRUCT_ATTRIBUTE_STR).substitute(var_type=var['type'],
                                                                                    var_identifier=var['identifier'],
                                                                                    default_value=default_value
                                                                                    )
                    result += CTemplate(ICE_STRUCT_STR).substitute(type_name=next_type['name'],
                                                                   attributes=attributes)
                if "sequence" == next_type["type"]:
                    result += Template(ICE_SEQUENCE_STR).substitute(type_sequence=next_type['typeSequence'],
                                                                    type_name=next_type['name'])
                if "dictionary" == next_type['type']:
                    result += Template(ICE_DICTIONARY_STR).substitute(type_content=next_type['content'],
                                                                      type_name=next_type['name'])
                if "enum" == next_type['type']:
                    result += Template(ICE_ENUM_STR).substitute(type_name=next_type['name'],
                                                                type_content=next_type['content'])
        return result

    def ice_interfaces(self):
        result = ""
        if "interfaces" in self.module:
            for interface in self.module['interfaces']:
                methods = ""
                for method in interface['methods'].values():
                    param_str_a = ''
                    for p in method['params']:
                        # delim
                        if param_str_a == '':
                            delim = ''
                        else:
                            delim = ', '
                        # STR
                        if p['decorator'] != "none" and p['decorator'] != '':
                            param_str_a += delim + p['decorator'] + ' ' + p['type'] + ' ' + p['name']
                        else:
                            param_str_a += delim + p['type'] + ' ' + p['name']
                    exception = ""
                    if method['throws'] != "nothing":
                        exception += " throws "
                        for p in method['throws']:
                            # STR
                            exception += p
                    method_decorator = method['decorator']+" " if bool(method['decorator']) else ""
                    methods += Template(ICE_METHOD_STR).substitute(method_decorator=method_decorator,
                                                                   method_return=method['return'],
                                                                   method_name=method['name'],
                                                                   params_str_a=param_str_a,
                                                                   exception=exception)
                result += CTemplate(ICE_INTERFACE_STR).substitute(interface_name=interface['name'],
                                                                  interface_methods=methods)
        return result