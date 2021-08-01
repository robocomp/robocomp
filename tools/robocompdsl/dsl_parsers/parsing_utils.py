import os
import pyparsing
from collections import Counter, OrderedDict
from rich.console import Console
from rich.text import Text


console = Console()
def generate_recursive_imports(initial_idsls, include_directories=None):
    assert isinstance(initial_idsls, list), "initial_idsl, parameter must be a list, not %s" % str(type(initial_idsls))
    if include_directories is None:
        include_directories = []
    new_idsls = []
    for idsl_path in initial_idsls:
        idsl_basename = os.path.basename(idsl_path)
        include_directories = include_directories + IDSLPool.get_common_interface_dirs()
        # TODO: Replace by idsl_robocomp_path
        new_idsl_path = idsl_robocomp_path(idsl_basename, include_directories)
        from dsl_parsers.dsl_factory import DSLFactory
        try:
            imported_module = DSLFactory().from_file(new_idsl_path)  # IDSLParsing.gimmeIDSL(attempt)
        except pyparsing.ParseException as e:
            console.log(f"Parsing error in file {Text(new_idsl_path, style='red')} while generating recursive imports.")
            console.log(f"Exception info: {Text(e.args[2], style='red')} in line {e.lineno} of:\n{Text(e.args[0].rstrip(), styled='magenta')}")
            raise
        if imported_module is None:
            raise FileNotFoundError('generate_recursive_imports: Couldn\'t locate %s' % idsl_basename)

        # if importedModule['imports'] have a # at the end an emtpy '' is generated
        idsl_imports = imported_module['imports']
        # we remove all the '' ocurrences and existing imports
        aux_imports = []
        for i_import in idsl_imports:
            if i_import != '' and i_import not in initial_idsls:
                if communication_is_ice(i_import):
                    aux_imports.append(i_import)
        idsl_imports = aux_imports
        if len(idsl_imports) > 0 and idsl_imports[0] != '':
            new_idsls += idsl_imports + generate_recursive_imports(idsl_imports, include_directories)

    return list(set(new_idsls))


def communication_is_ice(sb):
    is_ice = True
    if isinstance(sb, str):
        is_ice = True
    elif isinstance(sb, (list, tuple)):
        if len(sb) == 2:
            if sb[1] == 'ros'.lower():
                is_ice = False
            elif sb[1] != 'ice'.lower():
                console.log('Only ICE and ROS are supported', style='yellow')
                raise ValueError("Communication not ros and not ice, but %s" % sb[1])
    else:
        raise ValueError("Parameter %s of invalid type %s" % (str(sb), str(type(sb))))
    return is_ice


def is_valid_pubsub_idsl(idsl):
    for interface in idsl['interfaces']:
        if len(interface["methods"]) != 1:
            return False
        for method in interface["methods"].values():
            if method["return"] != "void":
                return False
            for param in method["params"]:
                if param["decorator"] == "out":
                    return False
    return True


def is_valid_rpc_idsl(idsl):
    for interface in idsl['interfaces']:
        if len(interface["methods"]) > 0:
            return True
    return False


def is_agm_agent(component):
    assert isinstance(component, (dict, OrderedDict)), \
        "Component parameter is expected to be a dict or OrderedDict but %s" % str(type(component))
    options = component.options
    return 'agmagent' in [x.lower() for x in options]


def idsl_robocomp_path(idsl_name, include_directories=None):
    assert isinstance(idsl_name, str), "idsl_name parameter must be a string"
    assert include_directories is None or isinstance(include_directories, list), \
        "include_directories must be a list of strings not %s" % str(type(include_directories))
    if not idsl_name.endswith('.idsl'):
        idsl_name += '.idsl'
    path_list = []
    if include_directories is not None:
        path_list += [x for x in include_directories]
    path_list += IDSLPool.get_common_interface_dirs()

    for p in path_list:
        path = os.path.join(p, idsl_name)
        if os.path.isfile(path):
            return path
    console.log(f"Couldn\'t locate {idsl_name}", style="yellow")
    return None


def get_name_number(names_list):
    """
    Used to add a number in case of multiple equal names
    :param names_list: list of names
    :return:
    """
    assert isinstance(names_list, list), "names_list must be a 'list' of interfaces (list) not %s" % str(type(names_list))
    for index, name in enumerate(names_list):
        assert isinstance(name, (list, tuple)), "names_list elements be a 'list' or tuple not %s" % str(type(names_list))

    ret = []
    c = Counter(names_list)
    keys = sorted(c)

    for k in keys:
        for cont in range(c[k]):
            if cont > 0:
                ret.append([k, str(cont)])
            else:
                ret.append([k, ''])
    return ret


def decorator_and_type_to_const_ampersand(decorator, vtype, module_pool, cpp11=False):
    ampersand = ' & '
    const = ' '
    if vtype in ['float', 'int', 'short', 'long', 'double']:  # MAIN BASIC TYPES
        if decorator in ['out']:  # out
            ampersand = ' &'
            const = ' '
        else:                      # read-only
            ampersand = ' '
            const = 'const '
    elif vtype in ['bool']:        # BOOL SEEM TO BE SPECIAL
        const = ' '
        if decorator in ['out']:  # out
            ampersand = ' &'
        else:                      # read-only
            ampersand = ' '
    elif vtype in ['string']:      # STRINGS
        if decorator in ['out']:  # out
            const = ' '
            ampersand = ' &'
        else:                      # read-only
            const = 'const '
            ampersand = ' &'
    else:                            # GENERIC, USED FOR USER-DEFINED DATA TYPES
        kind = get_kind_from_pool(vtype, module_pool)
        if kind is None:
            get_kind_from_pool(vtype, module_pool, debug=True)
            raise TypeError('error, unknown data structure, map or sequence '+vtype)
        else:
            if kind == 'enum':               # ENUM
                const = ' '
                if decorator in ['out']:  # out
                    ampersand = ' &'
                else:                      # read-only
                    ampersand = ' '
            else:                            # THE REST
                if decorator in ['out']:  # out
                    ampersand = ' &'
                    const = ' '
                else:                      # read-only
                    if not cpp11:
                        ampersand = ' &'
                        const = 'const '
                    else:
                        ampersand = ''
                        const = ''

    return const, ampersand


def get_kind_from_pool(vtype, module_pool, debug=False):
    if debug: console.log(vtype)
    split = vtype.split("::")
    if debug: print(split)
    if len(split) > 1:
        vtype = split[1]
        mname = split[0]
        if debug: console.log(('SPLIT (' + vtype+'), (' + mname + ')'))
        if mname in module_pool:
            if debug: console.log(('dentro SPLIT (' + vtype+'), (' + mname + ')'))
            r = get_type_from_module(vtype, module_pool[mname])
            if r is not None: return r
        if mname.startswith("RoboComp"):
            if mname[8:] in module_pool:
                r = get_type_from_module(vtype, module_pool[mname[8:]])
                if r is not None: return r
    else:
        if debug: console.log('no split')
        for module in module_pool:
            if debug: console.log(('  '+str(module)))
            r = get_type_from_module(vtype, module_pool[module])
            if r is not None: return r


def get_type_from_module(vtype, module):
    for t in module['types']:
        if t['name'] == vtype:
            return t['type']
    return None


FILE_PATH_DIR = os.path.dirname(os.path.realpath(__file__))
ALT_INTERFACES_DIR = os.path.join(FILE_PATH_DIR, "../../../interfaces/IDSLs/", )


class IDSLPool(OrderedDict):
    """
    This class is intended to load and store idsl modules from the corresponding files.
    idsl is the idsl filename or path
    module is the python structure loaded from an idsl file
    interfaces are the names defined for the communication inside idsl files and loaded in the modules.
    """
    mandatory_idsls = ["CommonBehavior.idsl"]

    common_interface_dirs = ['/opt/robocomp/interfaces/IDSLs/',
                             os.path.expanduser('~/robocomp/interfaces/IDSLs/'),
                             ALT_INTERFACES_DIR]

    def __init__(self, files, include_directories):
        super(IDSLPool, self).__init__()
        assert isinstance(files, list), "Files must be a list of strings"
        include_directories = include_directories + self.common_interface_dirs
        self.includeInPool(files, self, include_directories)
        self.includeInPool(self.mandatory_idsls, self, include_directories)
        self.module_inteface_check()

    @classmethod
    def get_common_interface_dirs(cls):
        return cls.common_interface_dirs

    def includeInPool(self, files, module_pool, include_directories):
        """
        Recursively add the loaded modules to the pool.

        """

        # look for the files in the includeDirectories
        for f in files:
            filename = f.split('.')[0]
            if filename not in module_pool:
                for p in include_directories:
                    try:
                        path = os.path.join(p, f)
                        # if found, load the module from the file
                        # WARN: import is here to avoid problem with recursive import on startup
                        from dsl_parsers.dsl_factory import DSLFactory
                        module = DSLFactory().from_file(path)
                        # store the module
                        module_pool[filename] = module
                        # try to add the modules that this one imports
                        self.includeInPool(module['imports'] + module['recursive_imports'], module_pool, include_directories)
                        break
                    except IOError:
                        pass
                if filename not in self:
                    raise ValueError('Couldn\'t locate %s ' % f)

    def IDSL_file_for_module(self, module):
        """
        Return the file path given the module object
        :param module: module to query on the pool for the related idsl file path
        :return: idsl file path
        """
        for filename in list(self.keys()):
            if self[filename] == module:
                return '/opt/robocomp/interfaces/IDSLs/'+filename+'.idsl'

    def module_providing_interface(self, interface):
        """
        Query the pool to get the module providing an interface
        :param interface: an interface to query the pool
        :return: the module providing the queried interface
        """
        for module in self:
            for m in self[module]['interfaces']:
                if m['name'] == interface:
                    return self[module]
        return None

    def module_inteface_check(self):
        for module in self:
            problem_found = True
            for m in self[module]['interfaces']:
                if m['name'] == os.path.splitext(os.path.basename(self[module]['filename']))[0]:
                    problem_found = False
                    break
            if problem_found:
                interface_names = []
                for m in self[module]['interfaces']:
                    interface_names.append(m['name'])
                console.log(f"WARNING: It's expected to find at least one interface with the name of the file."
                      f"\n\tExpected interface name <{os.path.splitext(os.path.basename(self[module]['filename']))[0]}> but only found "
                      f"<{', '.join(interface_names)}> in {self[module]['filename']}", style='red')

    def interfaces(self):
        """
        :return: a list of all the interfaces defined inside the modules
        """
        interfaces = []
        for module in self:
            for m in self[module]['interfaces']:
                interfaces.append(m['name'])
        return interfaces



if __name__ == '__main__':
    pool = IDSLPool("AGMCommonBehavior.idsl", [])
    print(pool["AGMCommonBehavior"]["imports"])
    pool = IDSLPool("AprilTags.idsl", [])
