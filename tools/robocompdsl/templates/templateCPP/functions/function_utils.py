

CPP_TYPES = ['void', 'char', 'unsigned char', 'signed char', 'short', 'int', 'unsigned int', 'signed int', 'short int',
             'unsigned short int', 'signed short int', 'long int', 'signed long int', 'unsigned long int', 'long',
             'unsigned long', 'float', 'double', 'long double', 'bool']

def get_parameters_string(method, module_name, language, servant=False):
    param_str = ""
    for p in method['params']:
        # delim
        if param_str == '':
            delim = ''
        else:
            delim = ', '
        # decorator
        ampersand = '&'
        if p['decorator'] == 'out':
            const = ''
        else:
            if language == "cpp":
                const = 'const '
            else:
                const = ''
                ampersand = ''
            if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                ampersand = ''
        # str
        param_type = get_type_string(p['type'], module_name, servant)
        param_str += delim + const + param_type + ' ' + ampersand + p['name']
    return param_str

def get_type_string(initial_type, module_name, servant=False):
    type_string = initial_type
    if type_string not in CPP_TYPES and '::' not in type_string:
        if type_string == 'string':
            type_string = f"std::{type_string}"
        else:
            type_string = f"{module_name}::{type_string}"
    elif type_string == "float" and servant:
        type_string = f"ice::{type_string}"
    return type_string
