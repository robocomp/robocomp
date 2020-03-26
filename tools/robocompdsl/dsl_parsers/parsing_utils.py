import os
import sys
import traceback
from collections import Counter, OrderedDict


def generate_recursive_imports(initial_idsls, include_directories=[]):
    assert isinstance(initial_idsls, list), "initial_idsl, parameter must be a list, not %s"%str(type(initial_idsls))
    new_idsls = []
    for idsl_path in initial_idsls:
        importedModule = None
        idsl_basename = os.path.basename(idsl_path)
        iD = include_directories + ['/opt/robocomp/interfaces/IDSLs/',
                                    os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
        # TODO: Replace by idsl_robocomp_path
        try:
            for directory in iD:
                attempt = directory + '/' + idsl_basename
                # print 'Check', attempt
                if os.path.isfile(attempt):
                    # WARN: import is here to avoid problem with recursive import on startup
                    from dsl_parsers.dsl_factory import DSLFactory
                    importedModule = DSLFactory().from_file(attempt)  # IDSLParsing.gimmeIDSL(attempt)
                    break
        except:
            print(('Error reading IMPORT', idsl_basename))
            traceback.print_exc()
            print(('Error reading IMPORT', idsl_basename))
            os._exit(1)
        if importedModule == None:
            print(('Counldn\'t locate', idsl_basename))
            os._exit(1)

        # if importedModule['imports'] have a # at the end an emtpy '' is generated
        idsl_imports = importedModule['imports'].split('#')
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
    isIce = True
    if isinstance(sb, str):
       isIce = True
    elif isinstance(sb, list):
        if len(sb) == 2:
            if sb[1] == 'ros'.lower():
                isIce = False
            elif sb[1] != 'ice'.lower() :
                print('Only ICE and ROS are supported')
                raise ValueError("Communication not ros and not ice, but %s"%sb[1])
    else:
        raise ValueError("Parameter %s of invalid type %s" %(str(sb), str(type(sb[1]))))
    return isIce

def is_agm1_agent(component):
    assert isinstance(component, (dict, OrderedDict)), \
        "Component parameter is expected to be a dict or OrderedDict but %s"%str(type(component))
    options = component['options']
    return 'agmagent' in [x.lower() for x in options]

def is_agm2_agent(component):
    assert isinstance(component, (dict, OrderedDict)), \
        "Component parameter is expected to be a dict or OrderedDict but %s" % str(type(component))
    valid = ['agm2agent', 'agm2agentros', 'agm2agentice']
    options = component['options']
    for v in valid:
        if v.lower() in options:
            return True
    return False

def is_agm2_agent_ROS(component):
    assert isinstance(component, (dict, OrderedDict)), \
        "Component parameter is expected to be a dict or OrderedDict but %s" % str(type(component))
    valid = ['agm2agentROS']
    options = component['options']
    for v in valid:
        if v.lower() in options:
            return True
    return False

def idsl_robocomp_path(idsl_name, include_directories = None):
    assert isinstance(idsl_name, str), "idsl_name parameter must be a string"
    assert include_directories is None or isinstance(include_directories, list), \
        "include_directories must be a list of strings not %s" % str(type(include_directories))
    pathList = []
    if include_directories != None:
        pathList += [x for x in include_directories]
    pathList.append('/opt/robocomp/interfaces/IDSLs/')
    pathList.append(os.path.expanduser('~/robocomp/interfaces/IDSLs/'))

    for p in pathList:
        path = os.path.join(p , idsl_name)
        if os.path.isfile(path):
            return path
    print(('Couldn\'t locate ', idsl_name))
    return None

def gimmeIDSL(name, files='', includeDirectories=None):
    assert not isinstance(includeDirectories, str), "includeDirectories must be a list of paths, not an str"
    if not '.idsl' in name:
        name += '.idsl'
    name = os.path.basename(name)
    pathList = []
    if includeDirectories!= None:
        pathList += [x for x in includeDirectories]
    fileList = []
    for p in [f for f in files.split('#') if len(f)>0]:
        if p.startswith("-I"):
            pathList.append(p[2:])
        else:
            fileList.append(p)
    pathList.append('/opt/robocomp/interfaces/IDSLs/')
    pathList.append(os.path.expanduser('~/robocomp/interfaces/IDSLs/'))
    filename = name.split('.')[0]
    for p in pathList:
        try:
            path = os.path.join(p,name)
            # WARN: import is here to avoid problem with recursive import on startup
            from dsl_parsers.dsl_factory import DSLFactory
            return DSLFactory().from_file(path)
        except IOError as e:
            pass
    print(('Couldn\'t locate ', name))
    sys.exit(-1)


def get_name_number(names_list):
    """
    Used add a number in case of multiple equal names
    :param names_list: list of names
    :return:
    """
    assert isinstance(names_list, list), "names_list must be a 'list' of names (str) not %s" % str(type(names_list))
    for name in names_list:
        assert isinstance(name, str), "names must be a 'str' not %s" % str(type(name))
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

def decoratorAndType_to_const_ampersand(decorator, vtype, modulePool, cpp11=False):
    ampersand = ' & '
    const = ' '

    if vtype in [ 'float', 'int', 'short', 'long', 'double' ]: # MAIN BASIC TYPES
        if decorator in [ 'out' ]: #out
            ampersand = ' &'
            const = ' '
        else:                      #read-only
            ampersand = ' '
            const = 'const '
    elif vtype in [ 'bool' ]:        # BOOL SEEM TO BE SPECIAL
        const = ' '
        if decorator in [ 'out' ]: # out
            ampersand = ' &'
        else:                      #read-only
            ampersand = ' '
    elif vtype in [ 'string' ]:      # STRINGS
        if decorator in [ 'out' ]: # out
            const = ' '
            ampersand = ' &'
        else:                      #read-only
            const = 'const '
            ampersand = ' &'
    else:                            # GENERIC, USED FOR USER-DEFINED DATA TYPES
        kind = getKindFromPool(vtype, modulePool)
        if kind == None:
            kind = getKindFromPool(vtype, modulePool, debug=True)
            raise Exception('error, unknown data structure, map or sequence '+vtype)
        else:
            if kind == 'enum':               # ENUM
                const = ' '
                if decorator in [ 'out' ]: # out
                    ampersand = ' &'
                else:                      #read-only
                    ampersand = ' '
            else:                            # THE REST
                if decorator in [ 'out' ]: # out
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

def getKindFromPool(vtype, modulePool, debug=False):
    if debug: print(vtype)
    split = vtype.split("::")
    if debug: print(split)
    if len(split) > 1:
        vtype = split[1]
        mname = split[0]
        if debug: print(('SPLIT (' + vtype+'), (' + mname + ')'))
        if mname in modulePool.modulePool:
            if debug: print(('dentro SPLIT (' + vtype+'), (' + mname + ')'))
            r = getTypeFromModule(vtype, modulePool.modulePool[mname])
            if r != None: return r
        if mname.startswith("RoboComp"):
            if mname[8:] in modulePool.modulePool:
                r = getTypeFromModule(vtype, modulePool.modulePool[mname[8:]])
                if r != None: return r
    else:
        if debug: print('no split')
        for module in modulePool.modulePool:
            if debug: print(('  '+str(module)))
            r = getTypeFromModule(vtype, modulePool.modulePool[module])
            if r != None: return r

def getTypeFromModule(vtype, module):
    for t in module['types']:
        if t['name'] == vtype:
            return t['type']
    return None


class IDSLPool:
    """
    This class is intended to load and store idsl modules from the corresponding files.
    idsl is the idsl filename or path
    module is the python structure loaded from an idsl file
    interfaces are the names defined for the communication inside idsl files and loaded in the modules.
    """

    rosTypes = (
    'int8', 'int16', 'int32', 'int64', 'float8', 'float16', 'float32', 'float64', 'byte', 'bool', 'string', 'time',
    'empty')

    def __init__(self, files, iD):
        self.modulePool = OrderedDict()
        includeDirectories = iD + ['/opt/robocomp/interfaces/IDSLs/', os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
        self.includeInPool(files, self.modulePool, includeDirectories)

    def getRosTypes(self):
        return self.rosTypes

    def includeInPool(self, files, modulePool, includeDirectories):
        """
        Recursively add the loaded modules to the pool.

        """
        fileList = []

        # Extracting files names from string argument "-I filename.idsl#filename2.idsl"
        for p in [f for f in files.split('#') if len(f)>0]:
            if p.startswith("-I"):
                pass
            else:
                fileList.append(p)

        # look for the files in the includeDirectories
        for f in fileList:
            filename = f.split('.')[0]
            if not filename in modulePool:
                for p in includeDirectories:
                    try:
                        path = os.path.join(p,f)
                        # if found, load the module from the file
                        # WARN: import is here to avoid problem with recursive import on startup
                        from dsl_parsers.dsl_factory import DSLFactory
                        module = DSLFactory().from_file(path)
                        # store the module
                        modulePool[filename] = module
                        # try to add the modules that this one imports
                        self.includeInPool(module['imports']+module['recursive_imports'], modulePool, includeDirectories)
                        break
                    except IOError as e:
                        pass
                if not filename in self.modulePool:
                    print(('Couldn\'t locate ', f))
                    # TODO: replace with an exception
                    sys.exit(-1)

    def IDSLsModule(self, module):
        """
        Return the file path given the module object
        :param module: module to query on the pool for the related idsl file path
        :return: idsl file path
        """
        for filename in list(self.modulePool.keys()):
            if self.modulePool[filename] == module:
                return '/opt/robocomp/interfaces/IDSLs/'+filename+'.idsl'

    def moduleProviding(self, interface):
        """
        Query the pool to get the module providing an interface
        :param interface: an interface to query the pool
        :return: the module providing the queried interface
        """
        for module in self.modulePool:
            for m in self.modulePool[module]['interfaces']:
                if m['name'] == interface:
                    return self.modulePool[module]
        return None

    def interfaces(self):
        """
        :return: a list of all the interfaces defined inside the modules
        """
        interfaces = []
        for module in self.modulePool:
            for m in self.modulePool[module]['interfaces']:
                interfaces.append(m['name'])
        return interfaces

    def rosImports(self):
        includesList = []
        for module in self.modulePool:
            for m in self.modulePool[module]['structs']:
                includesList.append(m['name'].split('/')[0]+"ROS/"+m['name'].split('/')[1])
            for m in self.modulePool[module]['sequences']:
                includesList.append(m['name'].split('/')[0]+"ROS/"+m['name'].split('/')[1])
            stdIncludes = {}
            for interface in self.modulePool[module]['interfaces']:
                for mname in interface['methods']:
                    method = interface['methods'][mname]
                    for p in method['params']:
                        if p['type'] in ('int','float'):
                            m = "std_msgs/"+p['type'].capitalize()+"32"
                            stdIncludes[p['type']] = m
                        elif p['type'] in ('uint8','uint16','uint32','uint64'):
                            m = "std_msgs/UInt"+p['type'].split('t')[1]
                        elif p['type'] in self.rosTypes:
                            m = "std_msgs/"+p['type'].capitalize()
                            stdIncludes[p['type']] = m
            for std in list(stdIncludes.values()):
                includesList.append(std)
        return includesList

    def rosModulesImports(self):
        modulesList = []
        for module in self.modulePool:
            for m in self.modulePool[module]['simpleStructs']:
                modulesList.append(m)
            for m in self.modulePool[module]['simpleSequences']:
                modulesList.append(m)
        return modulesList


if __name__ == '__main__':
    pool = IDSLPool("AGMCommonBehavior.idsl",[])
    print(pool.modulePool["AGMCommonBehavior"]["imports"])
    pool = IDSLPool("AprilTags.idsl", [])