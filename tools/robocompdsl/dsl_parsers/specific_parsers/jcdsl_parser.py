import json
import os
from collections import OrderedDict


from dsl_parsers.dsl_parser_abstract import DSLParserTemplate
from dsl_parsers.parsing_utils import is_agm2_agent, is_agm2_agent_ROS, communication_is_ice, is_agm1_agent, \
    generate_recursive_imports
from dsl_parsers.specific_parsers.componentinspections import ComponentInspections


class CDSLJsonParser(DSLParserTemplate):
    def __init__(self, include_directories=[]):
        super(CDSLJsonParser, self).__init__()
        self._include_directories = include_directories

    def _create_parser(self):
        pass

    def string_to_struct(self, dsl_string, **kwargs):
        component = OrderedDict(json.loads(dsl_string))
        inspections = ComponentInspections()
        inspections.check_all_inspections(component)

        # print 'parseCDSL.component', includeDirectories
        if self._include_directories is None:
            self._include_directories = []

        if "include_directories" in kwargs:
            self._include_directories = kwargs["include_directories"]

        if 'imports' in component:
            imprts = component['imports']
        else:
            imprts = []
        if is_agm1_agent(component):
            imprts.extend(['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl', 'AGMExecutiveTopic.idsl'])
        if is_agm2_agent(component):
            imprts.extend(['AGM2.idsl'])
        iD = self._include_directories + ['/opt/robocomp/interfaces/IDSLs/',
                                   os.path.expanduser('~/robocomp/interfaces/IDSLs/')]

        component['imports'] = list(map(os.path.basename, imprts))

        component['recursiveImports'] = generate_recursive_imports(list(component['imports']), self._include_directories)

        component['statemachine_visual'] = False
        if isinstance(component['statemachine'], list):
            if len(component['statemachine']) > 1:
                if component['statemachine'] == 'visual':
                    component['statemachine_visual'] = True

        try:
            component['innermodelviewer'] = 'innermodelviewer' in [x.lower() for x in component['options']]
        except:
            pass

        # TODO: Fix this ugly thing
        component["language"] = component["language"]["name"]

        com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
        for comm_type in com_types:
            if comm_type in component:
                for interface in component[comm_type]:
                    if communication_is_ice(interface):
                        component['iceInterfaces'].append(interface)
                    else:
                        component['rosInterfaces'].append(interface)
                        component['usingROS'] = True
        # Handle options for communications
        if is_agm1_agent(component):
            component['iceInterfaces'] += ['AGMCommonBehavior', 'AGMExecutive', 'AGMExecutiveTopic', 'AGMWorldModel']
            if not 'AGMCommonBehavior' in component['implements']:
                component['implements'] = ['AGMCommonBehavior'] + component['implements']
            if not 'AGMExecutive' in component['requires']:
                component['requires'] = ['AGMExecutive'] + component['requires']
            if not 'AGMExecutiveTopic' in component['subscribesTo']:
                component['subscribesTo'] = ['AGMExecutiveTopic'] + component['subscribesTo']
        if is_agm2_agent(component):
            if is_agm2_agent_ROS(component):
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
    file_path = "/home/robolab/robocomp/tools/robocompdsl/test/resources/jsoncomp.jcdsl"
    parser = CDSLJsonParser()
    with open(file_path, 'r') as reader:
        string = reader.read()
    struct = parser.string_to_struct(string)
    pass