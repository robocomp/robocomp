import os
from operator import itemgetter

from pyparsing import Suppress, Word, CaselessKeyword, alphas, alphanums, CharsNotIn, Group, ZeroOrMore, Optional, \
    delimitedList, cppStyleComment, ParseSyntaxException

from dsl_parsers.dsl_parser_abstract import DSLParserTemplate
from dsl_parsers.parsing_utils import communication_is_ice, generate_recursive_imports
from . import componentfacade


class CDSLParser(DSLParserTemplate):
    def __init__(self, include_directories=None):
        super(CDSLParser, self).__init__()
        if include_directories is None:
            include_directories = []
        self._include_directories = include_directories

    def _create_parser(self):
        OBRACE, CBRACE, SEMI, OPAR, CPAR = list(map(Suppress, "{};()"))
        QUOTE = Suppress(Word("\""))

        # keywords
        (IMPORT, COMMUNICATIONS, LANGUAGE, COMPONENT, CPP, CPP11, GUI, QWIDGET, QMAINWINDOW, QDIALOG, QT,
         PYTHON, REQUIRES, IMPLEMENTS, SUBSCRIBESTO, PUBLISHES, OPTIONS, TRUE, FALSE,
         INNERMODELVIEWER, STATEMACHINE, VISUAL, AGMAGENT, AGM2AGENT, AGM2AGENTICE, DSR, ICE, ROS) = list(map(CaselessKeyword, """
        import communications language component cpp cpp11 gui QWidget QMainWindow QDialog Qt 
        python requires implements subscribesTo publishes options true false
        InnerModelViewer statemachine visual agmagent agm2agent agm2agentice dsr ice ros""".split()))

        identifier = Word(alphas + "_", alphanums + "_")
        PATH = CharsNotIn("\";")

        # Imports
        idslImport = Group(Suppress(IMPORT) - QUOTE + PATH.setResultsName('idsl_path') - QUOTE + SEMI)

        idslImports = ZeroOrMore(idslImport).setResultsName("imports")

        commType = Optional(OPAR - (ICE | ROS).setResultsName("type") + CPAR, default='ice')

        implementsList = Optional(
            IMPLEMENTS - Group(delimitedList(Group(identifier.setResultsName("impIdentifier") + commType))).setResultsName(
                "implements") + SEMI)

        requiresList = Optional(
            REQUIRES - Group(delimitedList(Group(identifier.setResultsName("reqIdentifier") + commType))).setResultsName(
                "requires") + SEMI)

        subscribesList = Optional(
            SUBSCRIBESTO - Group(delimitedList(Group(identifier.setResultsName("subIdentifier") + commType))).setResultsName(
                "subscribesTo") + SEMI)

        publishesList = Optional(
            PUBLISHES - Group(delimitedList(Group(identifier.setResultsName("pubIdentifier") + commType))).setResultsName(
                "publishes") + SEMI)

        communicationList = Group(implementsList & requiresList & subscribesList & publishesList).setResultsName(
            "communications")
        communications = COMMUNICATIONS.suppress() - OBRACE + communicationList + CBRACE + SEMI

        # Language
        language_options = (CPP | CPP11 | PYTHON).setResultsName('language')
        language = LANGUAGE.suppress() - language_options - SEMI

        # GUI
        gui_options = QWIDGET | QMAINWINDOW | QDIALOG
        gui = Group(Optional(GUI.suppress() - QT('type') + OPAR - gui_options('gui_options') - CPAR + SEMI))
        # additional options
        valid_options = INNERMODELVIEWER | AGMAGENT | DSR
        options = Group(Optional(OPTIONS.suppress() - delimitedList(valid_options)) + SEMI)
        statemachine = Group(
            Optional(STATEMACHINE.suppress() - QUOTE + CharsNotIn("\";").setResultsName('machine_path') + QUOTE + Optional(VISUAL.setResultsName('visual').setParseAction(lambda t: True)) + SEMI))

        # Component definition
        componentContents = Group(
            communications - language + Optional(gui('gui')) + Optional(options('options')) + Optional(statemachine('statemachine'))).setResultsName(
            "content")
        component = Group(
            COMPONENT.suppress() - identifier("name") + OBRACE + componentContents + CBRACE + SEMI).setResultsName(
            "component")

        CDSL = idslImports - component
        CDSL.ignore(cppStyleComment)
        return CDSL

    def string_to_struct(self, string, **kwargs):
        try:
            parsing_result = self.parse_string(string)
        except ParseSyntaxException as e:
            raise ValueError("There was some problem parsing the component file.", e.args[1], e.args[2])
        component = componentfacade.ComponentFacade()
        # print 'parseCDSL.component', includeDirectories
        if self._include_directories is None:
            self._include_directories = []
        if "include_directories" in kwargs:
            self._include_directories = kwargs["include_directories"]
        # Set options
        component.options = []
        try:
            for op in parsing_result['component']['content']['options']:
                component.options.append(op.lower())
        except KeyError:
            pass

        # Component name
        component.name = parsing_result['component']['name']
        # Imports
        component.imports = []
        component.recursiveImports = []
        try:
            imprts = [path['idsl_path'] for path in parsing_result.asDict()["imports"]]
        except KeyError:
            parsing_result['imports'] = []
            imprts = []
        if component.is_agm1_agent():
            imprts.extend(['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl', 'AGMExecutiveTopic.idsl'])
        if component.is_agm2_agent():
            imprts.extend(['AGM2.idsl'])

        component.dsr = False
        component.dsr = 'dsr' in [x.lower() for x in component.options]
        if component.dsr:
            imprts.extend(['DSRGetID.idsl'])

        component.imports.extend(list(map(os.path.basename, sorted(imprts))))
        component.recursiveImports = generate_recursive_imports(list(component.imports), self._include_directories)
        # Language
        component.language = parsing_result['component']['content']['language']
        # Statemachine
        component.statemachine_path = None
        try:
            statemachine = parsing_result['component']['content']['statemachine']['machine_path']
            component.statemachine_path = statemachine
        except KeyError:
            pass

        try:
            statemachine_visual = parsing_result['component']['content']['statemachine']['visual']
        except KeyError:
            component.statemachine_visual = False
        else:
            component.statemachine_visual = statemachine_visual

        # innermodelviewer
        component.innermodelviewer = False
        component.innermodelviewer = 'innermodelviewer' in [x.lower() for x in component.options]


        # GUI
        component.gui = None
        try:
            ui_type = parsing_result['component']['content']['gui']['type']
            ui_widget = parsing_result['component']['content']['gui']['gui_options']
            if ui_type.lower() == 'qt' and ui_widget in ['QWidget', 'QMainWindow', 'QDialog']:
                component.gui = [ui_type, ui_widget]
            else:
                raise ValueError('Wrong UI specification %s' % parsing_result['properties']['gui'])
        except KeyError:
            pass

        # Communications
        component.rosInterfaces = []
        component.iceInterfaces = []
        component.implements = []
        component.requires = []
        component.publishes = []
        component.subscribesTo = []
        component.usingROS = False
        ####################
        com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
        communications = parsing_result['component']['content']['communications']
        for comm_type in com_types:
            if comm_type in communications:
                interfaces = sorted(communications[comm_type].asList(), key=itemgetter(0))
                for interface in interfaces:
                    getattr(component, comm_type).append(interface)
                    if communication_is_ice(interface):
                        component.iceInterfaces.append(interface)
                    else:
                        component.rosInterfaces.append(interface)
                        component.usingROS = True
        # Handle options for communications
        if component.is_agm1_agent():
            component.iceInterfaces += [['AGMCommonBehavior', 'ice'], ['AGMExecutive', 'ice'], ['AGMExecutiveTopic', 'ice'], ['AGMWorldModel', 'ice']]
            if 'AGMCommonBehavior' not in component.implements:
                component.implements = [['AGMCommonBehavior', 'ice']] + component.implements
            if 'AGMExecutive' not in component.requires:
                component.requires = [['AGMExecutive', 'ice']] + component.requires
            if 'AGMExecutiveTopic' not in component.subscribesTo:
                component.subscribesTo = [['AGMExecutiveTopic', 'ice']] + component.subscribesTo
        if component.is_agm2_agent():
            agm2agent_requires = [['AGMDSRService', 'ice']]
            agm2agent_subscribes_to = [['AGMExecutiveTopic', 'ice'], ['AGMDSRTopic', 'ice']]
            if 'AGMDSRService' not in component.iceInterfaces: component.iceInterfaces.append(['AGMDSRService', 'ice'])
            if 'AGMDSRTopic' not in component.iceInterfaces: component.iceInterfaces.append(['AGMDSRTopic', 'ice'])
            if 'AGMExecutiveTopic' not in component.iceInterfaces: component.iceInterfaces.append(['AGMExecutiveTopic', 'ice'])

            # AGM2 agents REQUIRES
            for agm2agent_req in agm2agent_requires:
                if agm2agent_req not in component.requires:
                    component.requires = [agm2agent_req] + component.requires
            # AGM2 agents SUBSCRIBES
            for agm2agent_sub in agm2agent_subscribes_to:
                if agm2agent_sub not in component.subscribesTo:
                    component.subscribesTo = [agm2agent_sub] + component.subscribesTo
        if component.dsr:
            component.requires = [["DSRGetID", 'ice']] + component.requires
            if 'DSRGetID' not in component.iceInterfaces:
                component.iceInterfaces.append(['DSRGetID', 'ice'])
        # component = ComponentFacade.from_nested_dict(component)
        self.struct = component
        return component

    def __str__(self):
        struct_str = ""
        struct_str += 'Component %s\n' % self.struct['name']

        struct_str += '\tImports:\n'
        for imp in self.struct['imports']:
            struct_str += '\t\t %s\n' % imp
        # Language
        struct_str += '\tLanguage:'
        struct_str += '\t\t %s\n' % self.struct['language']
        # GUI
        struct_str += '\tGUI:\n'
        struct_str += '\t\t %s\n' % self.struct['gui']
        # Communications
        struct_str += '\tCommunications:\n'
        struct_str += '\t\tImplements %s \n' % self.struct['implements']
        struct_str += '\t\tRequires %s\n' % self.struct['requires']
        struct_str += '\t\tPublishes %s\n' % self.struct['publishes']
        struct_str += '\t\tSubscribes %s\n' % self.struct['subscribesTo']
        return struct_str
