import json
import os

from dsl_parsers.dsl_parser_abstract import DSLParserTemplate
from dsl_parsers.parsing_utils import communication_is_ice, generate_recursive_imports
from dsl_parsers.specific_parsers.cdsl.componentinspections import ComponentInspections
from . import componentfacade


class CDSLJsonParser(DSLParserTemplate):
    def __init__(self, include_directories=[]):
        super(CDSLJsonParser, self).__init__()
        self._include_directories = include_directories


    def _create_parser(self):
        pass

    def string_to_struct(self, dsl_string, **kwargs):
        component = componentfacade.ComponentFacade.from_nested_dict(json.loads(dsl_string))
        inspections = ComponentInspections()
        inspections.check_all_inspections(component)

        # print 'parseCDSL.component', includeDirectories
        if self._include_directories is None:
            self._include_directories = []

        if "include_directories" in kwargs:
            self._include_directories = kwargs["include_directories"]

        if 'imports' in component:
            imprts = component.imports
        else:
            imprts = []
        if component.is_agm_agent():
            imprts.extend(['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl', 'AGMExecutiveTopic.idsl'])
        if component.is_agm_agent():
            imprts.extend(['AGM2.idsl'])
        iD = self._include_directories + ['/opt/robocomp/interfaces/IDSLs/',
                                   os.path.expanduser('~/robocomp/interfaces/IDSLs/')]

        component.imports = list(map(os.path.basename, imprts))

        component.recursiveImports = generate_recursive_imports(list(component.imports), self._include_directories)

        component.statemachine_visual = False
        if isinstance(component.statemachine, list):
            if len(component.statemachine) > 1:
                if component.statemachine == 'visual':
                    component.statemachine_visual = True

        try:
            component.innermodelviewer = 'innermodelviewer' in [x.lower() for x in component.options]
        except:
            pass

        # TODO: Fix this ugly thing
        component.language = component.language["name"]

        com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
        for comm_type in com_types:
            if comm_type in component:
                for interface in component[comm_type]:
                    if communication_is_ice(interface):
                        component.iceInterfaces.append(interface)
                    else:
                        component.rosInterfaces.append(interface)
                        component.usingROS = True
        # Handle options for communications
        if component.is_agm_agent():
            component.iceInterfaces += ['AGMCommonBehavior', 'AGMExecutive', 'AGMExecutiveTopic', 'AGMWorldModel']
            if not 'AGMCommonBehavior' in component.implements:
                component.implements = ['AGMCommonBehavior'] + component.implements
            if not 'AGMExecutive' in component.requires:
                component.requires = ['AGMExecutive'] + component.requires
            if not 'AGMExecutiveTopic' in component.subscribesTo:
                component.subscribesTo = ['AGMExecutiveTopic'] + component.subscribesTo
        self.struct = component
        return component

    def __str__(self):
        struct_str= ""
        struct_str+= 'Component %s\n' % self.struct['name']

        struct_str+= '\tImports:\n'
        for imp in self.struct['imports']:
            struct_str+= '\t\t %s\n'% imp
        # Language
        struct_str+= '\tLanguage:'
        struct_str+= '\t\t %s\n' %self.struct['language']
        # GUI
        struct_str+= '\tGUI:\n'
        struct_str+= '\t\t %\n' % self.struct['gui']
        # Communications
        struct_str+= '\tCommunications:\n'
        struct_str+= '\t\tImplements %s \n'% self.struct['implements']
        struct_str+= '\t\tRequires %s\n'% self.struct['requires']
        struct_str+= '\t\tPublishes %s\n'% self.struct['publishes']
        struct_str+= '\t\tSubscribes %s\n'% self.struct['subscribesTo']
        return struct_str


if __name__ == '__main__':
    file_path = "/test/resources/jsoncomp.jcdsl"
    parser = CDSLJsonParser()
    with open(file_path, 'r') as reader:
        string = reader.read()
    struct = parser.string_to_struct(string)
    pass