

PYTHON_TYPES = ['bool', 'float', 'int', 'long', 'str', 'double', 'byte']

def get_parameters_string(method, module_name, language):
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
        param_type = get_type_string(p['type'], module_name)
        param_str += delim + const + param_type + ' ' + ampersand + p['name']
    return param_str

def get_type_string(initial_type, module_name):
    type_string = initial_type
    if initial_type in PYTHON_TYPES:
        type_string = f"{initial_type}"
    elif initial_type in ['byte']:
        type_string = "bytes"
    elif initial_type in ['string']:
        type_string = "str"
    else:
        type_string = f"{module_name}.{initial_type}"

    return type_string
