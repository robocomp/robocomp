import json
import os
import traceback
from collections import OrderedDict
from difflib import SequenceMatcher
from functools import reduce

from pyparsing import Suppress, Word, CaselessKeyword, alphas, alphanums, CharsNotIn, Group, ZeroOrMore, Optional, \
    delimitedList, cppStyleComment
from termcolor import cprint

from dsl_parsers.dsl_parser_abstract import DSLParserTemplate
from dsl_parsers.parsing_utils import is_agm2_agent, is_agm2_agent_ROS, communication_is_ice, is_agm1_agent, \
    generate_recursive_imports

class ComponentInspections:
    def __init__(self):
        self.inspections = [
            {
                "function": "check_exists_or_fail",
                "object_path": ["name"],
                "params":
                    {
                        "message": "Name is mandatory for a component"
                    }
            },
            {
                "function": "check_valid_keys",
                "object_path": [],
                "params":
                    {
                        "keys": [
                            'imports',
                            'name',
                            'subscribesTo',
                            'options',
                            'recursiveImports',
                            'rosInterfaces',
                            'iceInterfaces',
                            'implements',
                            'requires',
                            'publishes',
                            'usingROS',
                            'innermodelviewer',
                            'language',
                            'name',
                            'modules',
                            'gui',
                            'statemachine'
                        ]
                    }
            },
            {
                "function": "check_exists_or_create",
                "object_path": ["implements"],
                "params":
                    {
                    "value": []
                    }
            },
            {
                "function": "check_exists_or_create",
                "object_path": ["innermodelviewer"],
                "params":
                    {
                        "value": False
                    }
            },
            {
                "function": "check_if",
                "object_path": [],
                "message": "You introduced modules not valid for Python",
                "params":
                    {
                        "condition":
                            {
                                "function": "check_value",
                                "object_path": ['language', 'name'],
                                "params":
                                    {
                                        'value': 'python'
                                    }
                            },
                        "true":
                            {
                                "function": "check_list_values_in",
                                "object_path": ['language', 'modules'],
                                "params":
                                    {
                                        'values': ['opencv']
                                    }
                            }
                    }
            }
        ]

    def check_all_inspections(self, object):
        for inspection in self.inspections:
            method = getattr(self,inspection['function'])
            result = method(object, inspection['object_path'], inspection['params'])
            if not result:
                if 'message' in inspection:
                    message = inspection['message']
                else:
                    message = str(inspection)
                cprint("NOT PASSED: %s" % message)

    def check_exists(self, object, object_path, params):
        if reduce(dict.get, object_path, object) is None:
            return False
        else:
            return True

    def check_exists_or_fail(self, object, object_path, params):
        if not self.check_exists(object, object_path, params):
            raise RuntimeError(params['message'])
        return True

    def check_exists_or_create(self, object, object_path, params):
        if reduce(dict.get, object_path, object) is None:
            for key in object_path[:-1]:
                if not key in object:
                    object[key] = {}
            pointer = reduce(dict.get, object_path[:-1], object)
            pointer[object_path[-1]] = params['value']
        return True

    def check_valid_keys(self, object, object_path, params):
        nested_object = reduce(dict.get, object_path, object)
        for key in nested_object.keys():
            if key not in params['keys']:
                path = "".join(["[\'%s\']"% key for key in object_path+[key]])
                cprint("\"%s\" is not a valid key for Component%s" % (key, path), "red")
                best_match = self.__find_best_match(key, params['keys'])
                if best_match is not None:
                    new_path = "".join(["[\'%s\']" % key for key in object_path + [best_match]])
                    cprint("May be you want to say Component%s" % new_path, "red")
                return False
        if isinstance(nested_object[key], dict):
            return self.check_valid_keys(object, object_path+[key], params)
        return True

    def check_value(self, object, object_path, params):
        value = reduce(dict.get, object_path, object)
        if value != params['value']:
            return False
        else:
            return True

    def check_list_values_in(self, object, object_path, params):
        list_to_check = reduce(dict.get, object_path, object)
        return all([x in params['values'] for x in list_to_check])

    def check_if(self,object, object_path, params):
        nested_object = reduce(dict.get, object_path, object)
        condition_method = getattr(self, params['condition']['function'])
        if condition_method(nested_object, params['condition']['object_path'], params['condition']['params']):
            if 'true' in params:
                true_method = getattr(self, params['true']['function'])
                return true_method(nested_object, params['true']['object_path'], params['true']['params'])
            return True
        else:
            if 'false' in params:
                false_method = getattr(self, params['false']['function'])
                return false_method(nested_object, params['false']['object_path'], params['false']['params'])
            return False

    def __find_best_match(self, reference, keys):
        max_ratio = -1
        best_match = None
        for key in keys:
            ratio = SequenceMatcher(None, reference, key).ratio()
            if 0.5 < ratio > max_ratio:
                max_ratio = ratio
                best_match = key
        return best_match






class CDSLJsonParser(DSLParserTemplate):
    def __init__(self, include_directories = []):
        super(CDSLJsonParser, self).__init__()
        self._include_directories = include_directories

    def _create_parser(self):
        pass

    def string_to_struct(self, string, **kwargs):
        component = OrderedDict(json.loads(string))
        # print 'parseCDSL.component', includeDirectories
        if self._include_directories == None:
            self._include_directories = []
        if "include_directories" in kwargs:
            self._include_directories = kwargs["include_directories"]

        if 'name' not in component:
            raise RuntimeError()

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
        # # Language
        # component['language'] = parsing_result['component']['content']['language']
        # Statemachine
        if 'statemachine' not in component:
            component['statemachine'] = None
        # component['statemachine'] = None
        # try:
        #     statemachine = parsing_result['component']['content']['statemachine']['machine_path']
        #     component['statemachine'] = statemachine
        # except:
        #     pass
        component['statemachine_visual'] = False
        if isinstance(component['statemachine'], list):
            if len(component['statemachine']) > 1:
                if component['statemachine'] == 'visual':
                    component['statemachine_visual'] = True


        # innermodelviewer
        component['innermodelviewer'] = False
        try:
            component['innermodelviewer'] = 'innermodelviewer' in [x.lower() for x in component['options']]
        except:
            pass
        if 'gui' not in component:
            component['gui'] = None
        # GUI
        # component['gui'] = None
        # try:
        #     uiT = parsing_result['component']['content']['gui']['type']
        #     uiI = parsing_result['component']['content']['gui']['gui_options']
        #     if uiT.lower() == 'qt' and uiI in ['QWidget', 'QMainWindow', 'QDialog']:
        #         component['gui'] = [uiT, uiI]
        #         pass
        #     else:
        #         raise ValueError('Wrong UI specification %s' % parsing_result['properties']['gui'])
        # except:
        #     # TODO: check exceptions and do something when accessing gui options fails.
        #     pass

        # Communications
        component['rosInterfaces'] = []
        component['iceInterfaces'] = []
        if 'implements' not in component: component['implements'] = []
        if 'requires' not in component: component['requires'] = []
        if 'publishes' not in component: component['publishes'] = []
        if 'subscribesTo' not in component: component['subscribesTo'] = []
        component['usingROS'] = False
        ####################
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
    inspections = ComponentInspections()
    struct['language'] = {"name": 'python', "modules": ["opencv"]}
    inspections.check_all_inspections(struct)
    pass