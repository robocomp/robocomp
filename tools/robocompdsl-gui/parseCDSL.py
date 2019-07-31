#!/usr/bin/env python

from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, CaselessLiteral, CaselessKeyword, ParseBaseException
from collections import Counter
import sys, traceback, os

from CDSLDocument import CDSLDocument

debug = False
# debug = True

from parseIDSL import *
import rcExceptions


def getTypeFromModule(vtype, module):
    for t in module['types']:
        if t['name'] == vtype:
            return t['type']
    return None


def getKindFromPool(vtype, modulePool, debug=False):
    if debug: print(vtype)
    split = vtype.split("::")
    if debug: print(split)
    if len(split) > 1:
        vtype = split[1]
        mname = split[0]
        if debug: print('SPLIT (' + vtype + '), (' + mname + ')')
        if mname in modulePool.modulePool:
            if debug: print('dentro SPLIT (' + vtype + '), (' + mname + ')')
            r = getTypeFromModule(vtype, modulePool.modulePool[mname])
            if r != None: return r
        if mname.startswith("RoboComp"):
            if mname[8:] in modulePool.modulePool:
                r = getTypeFromModule(vtype, modulePool.modulePool[mname[8:]])
                if r != None: return r
    else:
        if debug: print('no split')
        for module in modulePool.modulePool:
            if debug: print('  ' + str(module))
            r = getTypeFromModule(vtype, modulePool.modulePool[module])
            if r != None: return r


def decoratorAndType_to_const_ampersand(decorator, vtype, modulePool, cpp11=False):
    ampersand = ' & '
    const = ' '

    if vtype in ['float', 'int', 'short', 'long', 'double']:  # MAIN BASIC TYPES
        if decorator in ['out']:  # out
            ampersand = ' &'
            const = ' '
        else:  # read-only
            ampersand = ' '
            const = 'const '
    elif vtype in ['bool']:  # BOOL SEEM TO BE SPECIAL
        const = ' '
        if decorator in ['out']:  # out
            ampersand = ' &'
        else:  # read-only
            ampersand = ' '
    elif vtype in ['string']:  # STRINGS
        if decorator in ['out']:  # out
            const = ' '
            ampersand = ' &'
        else:  # read-only
            const = 'const '
            ampersand = ' &'
    else:  # GENERIC, USED FOR USER-DEFINED DATA TYPES
        kind = getKindFromPool(vtype, modulePool)
        if kind == None:
            kind = getKindFromPool(vtype, modulePool, debug=True)
            raise Exception('error, unknown data structure, map or sequence ' + vtype)
        else:
            if kind == 'enum':  # ENUM
                const = ' '
                if decorator in ['out']:  # out
                    ampersand = ' &'
                else:  # read-only
                    ampersand = ' '
            else:  # THE REST
                if decorator in ['out']:  # out
                    ampersand = ' &'
                    const = ' '
                else:  # read-only
                    if not cpp11:
                        ampersand = ' &'
                        const = 'const '
                    else:
                        ampersand = ''
                        const = ''

    return const, ampersand


def getNameNumber(aalist):
    ret = []
    c = Counter(aalist)
    keys = sorted(c)

    for k in keys:
        for cont in range(c[k]):
            if cont > 0:
                ret.append([k, str(cont)])
            else:
                ret.append([k, ''])
    return ret

class CDSLParsing:

    CDSLDoc = None  # attribute to store CDSLDocument

    def __init__(self, cdslDoc):
        self.CDSLDoc = cdslDoc

    def getCDSLDoc(self):
        return self.CDSLDoc

    def getCDSLParser(self):
        OBRACE, CBRACE, SEMI, OPAR, CPAR = map(Suppress, "{};()")
        QUOTE = Suppress(Word("\""))

        # keywords
        (
        IMPORT, COMMUNICATIONS, LANGUAGE, COMPONENT, CPP, CPP11, GUI, QT, PYTHON, REQUIRES, IMPLEMENTS,
        SUBSCRIBESTO, PUBLISHES, OPTIONS, TRUE, FALSE, InnerModelViewer, STATEMACHINE) = map(CaselessKeyword, """
        import communications language component cpp cpp11 gui Qt 
        python requires implements subscribesTo publishes options true false
        InnerModelViewer statemachine""".split())

        identifier = Word(alphas + "_", alphanums + "_")

        commIdentifier = Group(identifier('identifier') + Optional(
            OPAR + (CaselessKeyword("ice") | CaselessKeyword("ros")).setResultsName("type") + CPAR))

        # Imports
        idslImport = Suppress(IMPORT) - QUOTE + CharsNotIn("\";").setResultsName('path') - QUOTE + SEMI
        idslImports = ZeroOrMore(idslImport)

        # Communications
        implementsList = Group(IMPLEMENTS + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
        requiresList = Group(REQUIRES + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
        subscribesList = Group(SUBSCRIBESTO + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
        publishesList = Group(PUBLISHES + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
        communicationList = implementsList | requiresList | subscribesList | publishesList
        communications = Group(COMMUNICATIONS.suppress() + OBRACE + ZeroOrMore(communicationList) + CBRACE + SEMI)

        # Language
        language = Group(LANGUAGE.suppress() - (CPP | CPP11 | PYTHON) - SEMI).setParseAction(self.CDSLDoc.analize_language )

        # InnerModelViewer
        innermodelviewer = Group(Optional(InnerModelViewer.suppress() + (TRUE | FALSE) + SEMI))

        # GUI
        gui = Group(Optional(GUI.suppress() - QT + OPAR - identifier - CPAR + SEMI))
        # additional options
        options = Group(Optional(OPTIONS.suppress() + identifier + ZeroOrMore(Suppress(Word(',')) + identifier) + SEMI))

        statemachine = Group(Optional(STATEMACHINE.suppress() + QUOTE + CharsNotIn("\";").setResultsName('path') + QUOTE + SEMI))

        # Component definition
        componentContents = communications('communications') + language('language') + Optional(gui('gui')) + Optional(
            options('options')) + Optional(innermodelviewer('innermodelviewer'))
        component = COMPONENT.suppress() + identifier("name") + OBRACE + componentContents("properties") + CBRACE + SEMI

        CDSL = idslImports("imports") - component("component")

        return CDSL

    def analizeCDSL(self, filename, verbose=False, includeDirectories=None):
        cdsl_content = {}
        errors = []
        if verbose:
            print('Verbose:', verbose)

        if includeDirectories is None:
            includeDirectories = []

        try:
            cdsl_content = self.fromFile(filename, verbose=verbose, includeDirectories=includeDirectories)
        except rcExceptions.ParseException as ex:
            errors.append((ex.line, ex.message))
        except rcExceptions.RobocompDslException as ex:
            errors.append((0, ex.message))

        print(errors)

     #TO TEST GUI
    def analizeText(self, inputText, verbose=False, includeDirectories=None):
        cdsl_content = {}
        errors = []
        if verbose:
            print('Verbose:', verbose)

        if includeDirectories is None:
            includeDirectories = []

        try:
            cdsl_content = self.fromString(inputText)
        except rcExceptions.ParseException as ex:
            errors.append((ex.line, ex.message))
        except rcExceptions.RobocompDslException as ex:
            errors.append((0, ex.message))
        except:
            errors.append((0, "Unknown"))
        return cdsl_content, errors

    def fromFile(self, filename, verbose=False, includeDirectories=[]):
        try:
            inputText = open(filename, 'r').read()
        except (FileNotFoundError, IOError) as ex:
            if verbose:
                traceback.print_exc()
            raise rcExceptions.RobocompDslException("Error opening file: " + filename + str(ex))
        except:
            raise rcExceptions.RobocompDslException("Unexpected error: " + sys.exc_info()[0])

        try:
            ret = self.fromString(inputText, includeDirectories=includeDirectories)
        except rcExceptions.ParseException as ex:
            if verbose:
                traceback.print_exc()
            raise ex
        ret['filename'] = filename
        return ret


    def fromString(self, inputText, verbose=False, includeDirectories=[]):
        text = nestedExpr("/*", "*/").suppress().transformString(inputText)

        CDSL = self.getCDSLParser()
        CDSL.ignore(cppStyleComment)
        try:
            tree = CDSL.parseString(text)
        except ParseBaseException as e:
            raise rcExceptions.ParseException(str(e), e.line, e.column)

        cdsl_content = {}
        try:
            cdsl_content = self.component(tree, includeDirectories=includeDirectories)
        except rcExceptions.RobocompDslException as ex:
            raise ex
        return cdsl_content

        # TODO: Check if we can use lru_cache decorator.

    def generateRecursiveImports(self, initial_idsls, include_directories):

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
                print('Error reading IMPORT', idsl_basename)
                raise rcExceptions.RobocompDslException("Error reading import file: " + idsl_basename)
            if importedModule == None:
                print('Counldn\'t locate', idsl_basename)
                raise rcExceptions.RobocompDslException('Counldn\'t locate: ' + idsl_basename)

            # if importedModule['imports'] have a # at the end an emtpy '' is generated
            idsl_imports = importedModule['imports'].split('#')
            # we remove all the '' ocurrences
            idsl_imports = list(filter(('').__ne__, idsl_imports))
            if len(idsl_imports) > 0 and idsl_imports[0] != '':
                new_idsls += idsl_imports + self.generateRecursiveImports(idsl_imports, include_directories)
        return list(set(new_idsls))

    def printComponent(self, component, start=''):
        # Component name
        print('Component', component['name'])
        # Imports
        print('\tImports:')
        for imp in component['imports']:
            print('\t\t', imp)
        # Language
        print('\tLanguage:')
        print('\t\t', component['language'])
        # GUI
        print('\tGUI:')
        print('\t\t', component['gui'])
        # Communications
        print('\tCommunications:')
        print('\t\tImplements', component['implements'])
        print('\t\tRequires', component['requires'])
        print('\t\tPublishes', component['publishes'])
        print('\t\tSubscribes', component['subscribesTo'])

    def component(self, tree, includeDirectories=[], start=''):
        component = {}
        # print 'parseCDSL.component', includeDirectories

        # Set options
        component['options'] = []
        for op in tree['properties']['options']:
            #component['options'].append(op.lower())
            component['options'].append(op)


        # Component name
        component['name'] = tree['component']['name']
        # Imports
        component['imports'] = []
        component['recursiveImports'] = []
        try:
            imprts = tree['imports']
        except:
            tree['imports'] = []
            imprts = []
        if isAGM1Agent(component):
            imprts = ['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl']
            for i in tree['imports']:
                if not i in imprts:
                    imprts.append(i)
        if isAGM2Agent(component):
            imprts = ['AGM2.idsl']
            for i in tree['imports']:
                if not i in imprts:
                    imprts.append(i)

        iD = includeDirectories + ['/opt/robocomp/interfaces/IDSLs/',
                                   os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
        for imp in sorted(imprts):
            import_basename = os.path.basename(imp)
            component['imports'].append(import_basename)

        try:
            component['recursiveImports'] = self.generateRecursiveImports(component['imports'], includeDirectories)
        except rcExceptions.RobocompDslException as ex:
            raise ex
        # Language
        component['language'] = tree['properties']['language'][0]

        # Statemachine
        component['statemachine'] = 'none'
        try:
            statemachine = tree['properties']['statemachine'][0]
            component['statemachine'] = statemachine
        except:
            pass

        # innermodelviewer
        component['innermodelviewer'] = 'false'
        try:
            component['innermodelviewer'] = 'innermodelviewer' in [x.lower() for x in component['options']]
            pass
        except:
            pass
        # GUI
        component['gui'] = 'none'
        try:
            uiT = tree['properties']['gui'][0]
            uiI = tree['properties']['gui'][1]
            if uiT.lower() == 'qt' and uiI in ['QWidget', 'QMainWindow', 'QDialog']:
                component['gui'] = [uiT, uiI]
                pass
            else:
                print('Wrong UI specification', tree['properties']['gui'])
                sys.exit(1)
        except:
            pass

        # Communications
        component['rosInterfaces'] = []
        component['iceInterfaces'] = []
        component['implements'] = []
        component['requires'] = []
        component['publishes'] = []
        component['subscribesTo'] = []
        component['usingROS'] = "None"
        ####################
        com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
        communications = sorted(tree['properties']['communications'], key=lambda x: x[0])
        for comm in communications:
            if comm[0] in com_types:
                comm_type = comm[0]
                interfaces = sorted(comm[1:], key=lambda x: x[0])
                for interface in interfaces:
                    component[comm_type].append(interface[0])
                    if communicationIsIce(interface):
                        component['iceInterfaces'].append(interface[0])
                    else:
                        component['rosInterfaces'].append(interface[0])
                        component['usingROS'] = True
        # Handle options for communications
        if isAGM1Agent(component):
            component['iceInterfaces'] += ['AGMCommonBehavior', 'AGMExecutive', 'AGMWorldModel']
            if not 'AGMCommonBehavior' in component['implements']:
                component['implements'] = ['AGMCommonBehavior'] + component['implements']
            if not 'AGMExecutive' in component['requires']:
                component['requires'] = ['AGMExecutive'] + component['requires']
            if not 'AGMExecutiveTopic' in component['subscribesTo']:
                component['subscribesTo'] = ['AGMExecutiveTopic'] + component['subscribesTo']
        if isAGM2Agent(component):
            if isAGM2AgentROS(component):
                component['usingROS'] = True
                agm2agent_requires = [['AGMDSRService', 'ros']]
                agm2agent_subscribesTo = [['AGMExecutiveTopic', 'ros'], ['AGMDSRTopic', 'ros']]
                if not 'AGMDSRService' in component['rosInterfaces']: component['rosInterfaces'].append('AGMDSRService')
                if not 'AGMDSRTopic' in component['rosInterfaces']: component['rosInterfaces'].append('AGMDSRTopic')
                if not 'AGMExecutiveTopic' in component['rosInterfaces']: component['rosInterfaces'].append(
                    'AGMExecutiveTopic')
            else:
                agm2agent_requires = [['AGMDSRService', 'ice']]
                agm2agent_subscribesTo = [['AGMExecutiveTopic', 'ice'], ['AGMDSRTopic', 'ice']]
                if not 'AGMDSRService' in component['iceInterfaces']: component['iceInterfaces'].append('AGMDSRService')
                if not 'AGMDSRTopic' in component['iceInterfaces']: component['iceInterfaces'].append('AGMDSRTopic')
                if not 'AGMExecutiveTopic' in component['iceInterfaces']: component['iceInterfaces'].append(
                    'AGMExecutiveTopic')

            # AGM2 agents REQUIRES
            for agm2agent_req in agm2agent_requires:
                if not agm2agent_req in component['requires']:
                    component['requires'] = [agm2agent_req] + component['requires']
            # AGM2 agents SUBSCRIBES
            for agm2agent_sub in agm2agent_subscribesTo:
                if not agm2agent_sub in component['subscribesTo']:
                    component['subscribesTo'] = [agm2agent_sub] + component['subscribesTo']
        return component


def communicationIsIce(sb):
    isIce = True
    if len(sb) == 2:
        if sb[1] == 'ros'.lower():
            isIce = False
        elif sb[1] != 'ice'.lower():
            print('Only ICE and ROS are supported')
            sys.exit(-1)
    return isIce


def isAGM1Agent(component):
    options = component['options']
    return 'agmagent' in [x.lower() for x in options]


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


if __name__ == '__main__':
    files = ["Comp1.cdsl", "error.cdsl", "Comp2.cdsl", "Comp3.cdsl", "Comp4.cdsl", "Comp5.cdsl", "Comp6.cdsl", "eleComp.cdsl"]
    cdslDOC = CDSLDocument()
    parsing = CDSLParsing(cdslDOC)
    parsing.analizeCDSL("cdslFiles/Comp1.cdsl")
    parsing.analizeCDSL("cdslFiles/Comp2.cdsl")

    sys.exit()
    for file in files:
        print(file)
        parsing.analizeCDSL("cdslFiles/" + file)
        print()

