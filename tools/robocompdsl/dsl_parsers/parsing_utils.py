import os

from parseIDSL import IDSLParsing


def generateRecursiveImports(initial_idsls, include_directories):
    new_idsls = []
    for idsl_path in initial_idsls:
        importedModule = None
        idsl_basename = os.path.basename(idsl_path)
        iD = include_directories + ['/opt/robocomp/interfaces/IDSLs/',
                                    os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
        try:
            for directory in iD:
                attempt = directory + '/' + idsl_basename
                # print 'Check', attempt
                if os.path.isfile(attempt):
                    importedModule = IDSLParsing.fromFile(attempt)  # IDSLParsing.gimmeIDSL(attempt)
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
                if communicationIsIce(i_import):
                    aux_imports.append(i_import)
        idsl_imports = aux_imports
        if len(idsl_imports) > 0 and idsl_imports[0] != '':
            new_idsls += idsl_imports + generateRecursiveImports(idsl_imports, include_directories)

    return list(set(new_idsls))

def communicationIsIce(sb):
	isIce = True
	if len(sb) == 2:
		if sb[1] == 'ros'.lower():
			isIce = False
		elif sb[1] != 'ice'.lower() :
			print('Only ICE and ROS are supported')
			sys.exit(-1)
	return isIce

def isAGM1Agent(component):
	options = component['options']
	return 'agmagent' in [ x.lower() for x in options]

def isAGM2Agent(component):
	valid = ['agm2agent', 'agm2agentros', 'agm2agentice']
	options = component['options']
	for v in valid:
		if v.lower() in options:
			return True
	return False

def isAGM2AgentROS(component):
	valid = ['agm2agentROS']
	options = component['options']
	for v in valid:
		if v.lower() in options:
			return True
	return False

def gimmeIDSL(name, files='', includeDirectories=None):
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
    pathList.append('/home/robocomp/robocomp/interfaces/IDSLs/')
    pathList.append('/opt/robocomp/interfaces/IDSLs/')
    filename = name.split('.')[0]
    for p in pathList:
        try:
            path = p+'/'+name
            return IDSLParsing.fromFile(path)
        except IOError as e:
            pass
    print(('Couldn\'t locate ', name))
    sys.exit(-1)

def gimmeIDSLStruct(name, files='', includeDirectories=None):
    pathList = []
    if includeDirectories!= None:
        pathList += [x for x in includeDirectories]
    fileList = []
    for p in [f for f in files.split('#') if len(f)>0]:
        if p.startswith("-I"):
            pathList.append(p[2:])
        else:
            fileList.append(p)
    pathList.append('/home/robocomp/robocomp/interfaces/IDSLs/')
    pathList.append('/opt/robocomp/interfaces/IDSLs/')
    filename = name.split('.')[0]
    for p in pathList:
        try:
            path = p+'/'+name
            return IDSLParsing.fromFileIDSL(path)
        except IOError as e:
            pass
    print(('Couldn\'t locate ', name))
    sys.exit(-1)