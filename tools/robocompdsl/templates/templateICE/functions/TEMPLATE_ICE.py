import os
from string import Template

from ...common.abstracttemplate import CustomTemplate as CTemplate

def ice_imports(module):
    result = ""
    if 'imports' in module and module["imports"] != '':
        for imp in module['imports'].split('#'):
            if imp != '':
                result += "#include <" + os.path.basename(imp).split('.')[0] + ".ice>"
    return result

ICE_EXCEPTIOM_STR = """\
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

def ice_types(module):
    result = ""
    if 'types' in module:
        for next_type in module["types"]:
            if "exception" == next_type["type"]:
                result += Template(ICE_EXCEPTIOM_STR).substitute(type_name=next_type['name'],
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

ICE_INTERFACE_STR= """\
interface ${interface_name}
{
	${interface_methods}
};
"""

ICE_METHOD_STR = """\
${method_decorator}${method_return} ${method_name} (${params_str_a})${exception};
"""

def ice_interfaces(module):
    result = ""
    if "interfaces" in module:
        for interface in module['interfaces']:
            methods = ""
            for method in interface['methods'].values():
                paramStrA = ''
                for p in method['params']:
                    # delim
                    if paramStrA == '':
                        delim = ''
                    else:
                        delim = ', '
                    # STR
                    if p['decorator'] != "none" and p['decorator'] != '':
                        paramStrA += delim + p['decorator'] + ' ' + p['type'] + ' ' + p['name']
                    else:
                        paramStrA += delim + p['type'] + ' ' + p['name']
                exception = ""
                if method['throws'] != "nothing":
                    exception +=" throws "
                    for p in method['throws']:
                        # STR
                        exception += p
                method_decorator = method['decorator']+" " if bool(method['decorator']) else ""
                methods += Template(ICE_METHOD_STR).substitute(method_decorator=method_decorator,
                                                               method_return=method['return'],
                                                               method_name=method['name'],
                                                               params_str_a=paramStrA,
                                                               exception=exception)
            result += CTemplate(ICE_INTERFACE_STR).substitute(interface_name=interface['name'],
                                                             interface_methods=methods)
    return result

def get_template_dict(module):
    return {
        'module_name': module['name'],
        'module_filename': os.path.basename(module['filename']).split('.')[0],
        'module_file': os.path.basename(module['filename']),
        'module_name_upper': module['name'].upper(),
        'ice_imports': ice_imports(module),
        'ice_types': ice_types(module),
        'ice_interfaces': ice_interfaces(module)
    }