import json
import os
import sys
import unittest
from collections import OrderedDict

from pyparsing import ParseException
sys.path.append("/opt/robocomp/python")
sys.path.append('/opt/robocomp/share/robocompdsl/')
import dsl_parsers.specific_parsers.cdsl.componentfacade as cf

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
RESOURCES_DIR = os.path.join(CURRENT_DIR, "resources")
sys.path.append(ROBOCOMPDSL_DIR)

from dsl_parsers.dsl_factory import DSLFactory




def deep_sort(obj):
    """
    Recursively sort list or dict nested lists
    """

    if isinstance(obj, (dict, OrderedDict)):
        _sorted = {}
        for key in sorted(obj):
            # try:
            _sorted[key] = deep_sort(obj[key])
            # except:
            #     _sorted[key] = deep_sort(getattr(obj, key))


    elif isinstance(obj, list):
        new_list = []
        for val in obj:
            new_list.append(deep_sort(val))
        _sorted = sorted(new_list)

    else:
        _sorted = obj

    return _sorted

class DSLFactoryTestCase(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None
        self.factory = DSLFactory()

    def assertNestedDictEqual(self, first, second, msg=None):
        j1 = json.dumps(first, sort_keys=True, indent=4)
        j2 = json.dumps(second, sort_keys=True, indent=4)
        self.maxDiff = None
        # with open('last_json1.txt', 'w') as outfile:
        #     json.dump(first, outfile, sort_keys=True, indent=4)
        # with open('last_json2.txt', 'w') as outfile:
        #     json.dump(second, outfile, sort_keys=True, indent=4)
        self.assertEqual(j1, j2, msg)

    def test_factory_singleton(self):
        factory2 = DSLFactory()
        self.assertIs(self.factory, factory2)

    def test_factory_smdsl(self):
        a = self.factory.from_file(os.path.join(RESOURCES_DIR, "gamestatemachine.smdsl"))
        self.assertDictEqual(a, {
            'machine': {
                'name': 'application_machine',
                'default': False,
                'contents': {
                    'states': None,
                    'finalstate': 'app_end',
                    'initialstate': 'game_machine',
                    'transitions': [
                        {'src': 'game_machine', 'dests': ['app_end']}]
                }
            },
            'substates': [
                {
                    'parallel': False,
                    'parent': 'game_machine',
                    'contents': {
                        'states': [
                            'session_init',
                            'game_start_wait',
                            'game_init',
                            'game_loop',
                            'game_pause',
                            'game_resume',
                            'game_reset',
                            'game_end',
                            'game_won',
                            'game_lost',
                            'session_end'],
                        'finalstate': None,
                        'initialstate': 'session_start_wait',
                        'transitions': None
                    }
                },
                {
                    'parallel': False,
                    'parent': 'session_init',
                    'contents': {
                        'states': [
                            'player_acquisition_loop'],
                        'finalstate': 'player_acquisition_ended',
                        'initialstate': 'player_acquisition_init',
                        'transitions': None
                    }
                }
            ],
            'filename': os.path.join(RESOURCES_DIR, "gamestatemachine.smdsl")
        })
        # Test for cached query
        b = self.factory.from_file(os.path.join(RESOURCES_DIR, "gamestatemachine.smdsl"))
        self.assertIs(a, b)

    def test_factory_idsl(self):
        c = self.factory.from_file("/opt/robocomp/interfaces/IDSLs/JointMotor.idsl")
        ref = OrderedDict({
            'name': 'RoboCompJointMotor',
            'imports': '',
            'recursive_imports': '',
            'interfaces':[
                    {'name': 'JointMotor',
                     'methods':
                         OrderedDict([
                             ('getAllMotorParams',{'name': 'getAllMotorParams', 'decorator': '', 'return': 'MotorParamsList', 'params': [], 'throws': 'nothing'}),
                             ('getAllMotorState', {'name': 'getAllMotorState', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'out', 'type': 'MotorStateMap', 'name': 'mstateMap'}], 'throws': ['UnknownMotorException']}),
                             ('getBusParams', {'name': 'getBusParams', 'decorator': '', 'return': 'BusParams', 'params': [], 'throws': 'nothing'}),
                             ('getMotorParams', {'name': 'getMotorParams', 'decorator': '', 'return': 'MotorParams', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'motor'}], 'throws': ['UnknownMotorException']}),
                             ('getMotorState', {'name': 'getMotorState', 'decorator': '', 'return': 'MotorState', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'motor'}], 'throws': ['UnknownMotorException']}),
                             ('getMotorStateMap', {'name': 'getMotorStateMap', 'decorator': '', 'return': 'MotorStateMap', 'params': [{'decorator': 'none', 'type': 'MotorList', 'name': 'mList'}], 'throws': ['UnknownMotorException']}),
                             ('setPosition', {'name': 'setPosition', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'MotorGoalPosition', 'name': 'goal'}], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException', ',', 'CollisionException']}),
                             ('setSyncPosition', {'name': 'setSyncPosition', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'MotorGoalPositionList', 'name': 'listGoals'}], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException']}),
                             ('setSyncVelocity', {'name': 'setSyncVelocity', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'MotorGoalVelocityList', 'name': 'listGoals'}], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException']}),
                             ('setSyncZeroPos', {'name': 'setSyncZeroPos', 'decorator': '', 'return': 'void', 'params': [], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException']}),
                             ('setVelocity', {'name': 'setVelocity', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'MotorGoalVelocity', 'name': 'goal'}], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException']}),
                             ('setZeroPos', {'name': 'setZeroPos', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'name'}], 'throws': ['UnknownMotorException', ',', 'HardwareFailedException']})])},
                    {'name': 'JointMotorPublish',
                     'methods':
                         OrderedDict([
                             ('motorStates', {'name': 'motorStates',
                                              'decorator': '',
                                              'return': 'void',
                                              'params': [
                                                  {'decorator': 'none',
                                                   'type': 'MotorStateMap',
                                                   'name': 'mstateMap'}
                                              ],
                                              'throws': 'nothing'
                                              })
                         ])
                     }
                ],
            'types': [
                {'type': 'exception',
                 'name': 'HardwareFailedException',
                 'content': '\n                string  what;\n        '},
                {'type': 'exception',
                 'name': 'OutOfRangeException',
                 'content': '\n                string  what;\n        '},
                {'type': 'exception',
                 'name': 'UnknownMotorException',
                 'content': '\n                string  what;\n        '},
                {'type': 'exception',
                 'name': 'CollisionException',
                 'content': '\n                string what;\n        '},
                {'type': 'struct',
                 'name': 'MotorState',
                 'structIdentifiers': [{'type': 'float', 'identifier': 'pos'},
                                       {'type': 'float', 'identifier': 'vel'},
                                       {'type': 'float', 'identifier': 'power'},
                                       {'type': 'string', 'identifier': 'timeStamp'},
                                       {'type': 'int', 'identifier': 'p'},
                                       {'type': 'int', 'identifier': 'v'},
                                       {'type': 'bool', 'identifier': 'isMoving'},
                                       {'type': 'int', 'identifier': 'temperature'}]},
                {'type': 'dictionary',
                 'content': 'string,MotorState',
                 'name': 'MotorStateMap'},
                {'type': 'struct',
                 'name': 'MotorParams',
                 'structIdentifiers': [{'type': 'string', 'identifier': 'name'},
                                       {'type': 'byte', 'identifier': 'busId'},
                                       {'type': 'float', 'identifier': 'minPos'},
                                       {'type': 'float', 'identifier': 'maxPos'},
                                       {'type': 'float', 'identifier': 'maxVelocity'},
                                       {'type': 'float', 'identifier': 'zeroPos'},
                                       {'type': 'float', 'identifier': 'stepsRange'},
                                       {'type': 'float', 'identifier': 'maxDegrees'},
                                       {'type': 'bool', 'identifier': 'invertedSign'},
                                       {'type': 'float', 'identifier': 'offset'},
                                       {'type': 'float', 'identifier': 'unitsRange'}]},
                {'type': 'sequence',
                 'typeSequence': 'MotorParams',
                 'name': 'MotorParamsList'},
                {'type': 'struct',
                 'name': 'BusParams',
                 'structIdentifiers': [{'type': 'string', 'identifier': 'handler'},
                                       {'type': 'string', 'identifier': 'device'},
                                       {'type': 'int', 'identifier': 'numMotors'},
                                       {'type': 'int', 'identifier': 'baudRate'},
                                       {'type': 'int', 'identifier': 'basicPeriod'}]},
                {'type': 'struct',
                 'name': 'MotorGoalPosition',
                 'structIdentifiers': [{'type': 'string', 'identifier': 'name'},
                                       {'type': 'float', 'identifier': 'position'},
                                       {'type': 'float', 'identifier': 'maxSpeed'}]},
                {'type': 'sequence',
                 'typeSequence': 'MotorGoalPosition',
                 'name': 'MotorGoalPositionList'},
                {'type': 'struct',
                 'name': 'MotorGoalVelocity',
                 'structIdentifiers': [{'type': 'string', 'identifier': 'name'},
                                       {'type': 'float', 'identifier': 'velocity'},
                                       {'type': 'float', 'identifier': 'maxAcc'}]},
                {'type': 'sequence',
                 'typeSequence': 'MotorGoalVelocity',
                 'name': 'MotorGoalVelocityList'},
                {'type': 'sequence',
                 'typeSequence': 'string',
                 'name': 'MotorList'}],
            'sequences': [
                {'name': 'RoboCompJointMotor/MotorParamsList',
                 'type': 'sequence',
                 'typeSequence': 'MotorParams'},
                {'name': 'RoboCompJointMotor/MotorGoalPositionList',
                 'type': 'sequence',
                 'typeSequence': 'MotorGoalPosition'},
                {'name': 'RoboCompJointMotor/MotorGoalVelocityList',
                 'type': 'sequence',
                 'typeSequence': 'MotorGoalVelocity'},
                {'name': 'RoboCompJointMotor/MotorList',
                 'type': 'sequence',
                 'typeSequence': 'string'}
            ],
            'simpleSequences': [
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorParamsList'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorGoalPositionList'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorGoalVelocityList'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorList'}
            ],
            'structs': [
                {'name': 'RoboCompJointMotor/MotorState',
                 'type': 'struct',
                 'structIdentifiers': [
                     ['float', 'pos'],
                     ['float', 'vel'],
                     ['float', 'power'],
                     ['string', 'timeStamp'],
                     ['int', 'p'],
                     ['int', 'v'],
                     ['bool', 'isMoving'],
                     ['int', 'temperature']
                 ]
                 },
                {'name': 'RoboCompJointMotor/MotorParams',
                 'type': 'struct',
                 'structIdentifiers': [
                     ['string', 'name'],
                     ['byte', 'busId'],
                     ['float', 'minPos'],
                     ['float', 'maxPos'],
                     ['float', 'maxVelocity'],
                     ['float', 'zeroPos'],
                     ['float', 'stepsRange'],
                     ['float', 'maxDegrees'],
                     ['bool', 'invertedSign'],
                     ['float', 'offset'],
                     ['float', 'unitsRange']
                 ]
                 },
                {'name': 'RoboCompJointMotor/BusParams',
                 'type': 'struct',
                 'structIdentifiers': [
                     ['string', 'handler'],
                     ['string', 'device'],
                     ['int', 'numMotors'],
                     ['int', 'baudRate'],
                     ['int', 'basicPeriod']
                 ]
                 },
                {'name': 'RoboCompJointMotor/MotorGoalPosition',
                 'type': 'struct',
                 'structIdentifiers': [
                     ['string', 'name'],
                     ['float', 'position'],
                     ['float', 'maxSpeed']
                 ]
                 },
                {'name': 'RoboCompJointMotor/MotorGoalVelocity',
                 'type': 'struct',
                 'structIdentifiers': [
                     ['string', 'name'],
                     ['float', 'velocity'],
                     ['float', 'maxAcc']
                 ]
                 }
            ],
            'simpleStructs': [
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorState'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorParams'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'BusParams'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorGoalPosition'},
                {'name': 'RoboCompJointMotor',
                 'strName': 'MotorGoalVelocity'}
            ],
            'filename': '/opt/robocomp/interfaces/IDSLs/JointMotor.idsl'
        })
        self.assertNestedDictEqual(c, ref)
        # test for cached query
        d = self.factory.from_file("/opt/robocomp/interfaces/IDSLs/JointMotor.idsl")
        self.assertIs(c, d)

    def test_factory_cdsl(self):
        # TODO: Use a better cdsl example than this
        g = self.factory.from_file(os.path.join(RESOURCES_DIR, "customstatemachinecpp.cdsl"))
        ref = cf.ComponentFacade({
            'options': [],
            'name': 'testcomp',
            'imports': [],
            'recursiveImports': [],
            'language': 'cpp',
            'statemachine_path': 'gamestatemachine.smdsl',
            'statemachine_visual': False,
            'innermodelviewer': False,
            'gui': ['Qt', 'QWidget'],
            'rosInterfaces': [],
            'iceInterfaces': [],
            'implements': [],
            'requires': [],
            'publishes': [],
            'subscribesTo': [],
            'usingROS': False,
            'filename': os.path.join(RESOURCES_DIR, "customstatemachinecpp.cdsl")})
        self.assertEqual(g, ref)
        # test for cached query
        h = self.factory.from_file(os.path.join(RESOURCES_DIR, "customstatemachinecpp.cdsl"))
        self.assertIs(g, h)

    def test_factory_cdsl_with_options(self):
        # TODO: Use a better cdsl example than this
        g = self.factory.from_file(os.path.join(RESOURCES_DIR, "componentwithoptions.cdsl"))
        ref = cf.ComponentFacade({
            'options': ['agmagent', 'innermodelviewer'],
            'name': 'testcomp',
            'imports': ['AGMCommonBehavior.idsl',
                        'AGMExecutive.idsl',
                        'AGMExecutiveTopic.idsl',
                        'AGMWorldModel.idsl'],
            'recursiveImports': ['Planning.idsl'],
            'language': 'cpp',
            'statemachine_path': None,
            'statemachine_visual': False,
            'innermodelviewer': True,
            'gui': ['Qt', 'QWidget'],
            'rosInterfaces': [],
            'iceInterfaces': [['AGMCommonBehavior', 'ice'],
                              ['AGMExecutive', 'ice'],
                              ['AGMExecutiveTopic', 'ice'],
                              ['AGMWorldModel', 'ice']],
            'implements': [['AGMCommonBehavior','ice']],
            'requires': [['AGMExecutive', 'ice']],
            'publishes': [],
            'subscribesTo': [['AGMExecutiveTopic', 'ice']],
            'usingROS': False,
            'filename': os.path.join(RESOURCES_DIR, "componentwithoptions.cdsl")})
        self.assertEqual(g, ref)

        # test for cached query
        h = self.factory.from_file(os.path.join(RESOURCES_DIR, "componentwithoptions.cdsl"))
        self.assertIs(g, h)


    # def test_factory_jcdsl(self):
    #     # TODO: Use a better cdsl example than this
    #     g = self.factory.from_file(os.path.join(RESOURCES_DIR, "jsoncomp.jcdsl"))
    #     self.assertCountEqual(g, {
    #         'options': ['agmagent', 'innermodelviewer'],
    #         'name': 'testcomp',
    #         'imports': ['AGMCommonBehavior.idsl',
    #                     'AGMExecutive.idsl',
    #                     'AGMExecutiveTopic.idsl',
    #                     'AGMWorldModel.idsl'],
    #         'recursiveImports': ['Planning.idsl'],
    #         'language': 'cpp',
    #         'statemachine': None,
    #         'statemachine_visual': False,
    #         'innermodelviewer': True,
    #         'gui': ['Qt', 'QWidget'],
    #         'rosInterfaces': [],
    #         'iceInterfaces': ['AGMCommonBehavior',
    #                           'AGMExecutive',
    #                           'AGMExecutiveTopic',
    #                           'AGMWorldModel'],
    #         'implements': ['AGMCommonBehavior'],
    #         'requires': ['AGMExecutive'],
    #         'publishes': [],
    #         'subscribesTo': ['AGMExecutiveTopic'],
    #         'usingROS': False,
    #         'filename': os.path.join(RESOURCES_DIR, "jsoncomp.jcdsl")})
    #     # test for cached query
    #     h = self.factory.from_file(os.path.join(RESOURCES_DIR, "jsoncomp.jcdsl"))
    #     self.assertIs(g, h)

    def test_factory_special_cases(self):
        # valid idsl without path
        a = self.factory.from_file("JointMotor.idsl")
        b = self.factory.from_file("/opt/robocomp/interfaces/IDSLs/JointMotor.idsl")
        self.assertEqual(a,b)

        # Not valid idsl without path
        self.assertRaises(IOError, self.factory.from_file,"NotAnIdsl.idsl")

        # None as path
        self.assertEqual(self.factory.from_file(None), None)

        # invalid dsl type from file extension
        self.assertRaises(ValueError, self.factory.from_string,"asdafafsdfasfsdfasdff", "ppsl")

        # invalid dsl type from file extension
        self.assertRaises(ParseException, self.factory.from_string, "this is not a valid string for the parser", "idsl")



    def assertFilesSame(self, sFName1, sFName2):
        text1 = open(os.path.join(self.tempdir, sFName1), 'rb').read()
        text2 = open(os.path.join(self.tempdir, sFName2), 'rb').read()
        self.assertEqual(text1, text2)

if __name__ == '__main__':
    unittest.main()
