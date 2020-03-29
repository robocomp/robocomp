import os
import sys
import unittest
from unittest import TestCase

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
sys.path.append(ROBOCOMPDSL_DIR)

from dsl_parsers import parsing_utils
from dsl_parsers.dsl_factory import DSLFactory


# noinspection PyCompatibility
class ParsingUtilsTest(unittest.TestCase):

    def test_generate_recursive_imports(self):
        self.assertRaises(AssertionError, parsing_utils.generate_recursive_imports, "AprilTags.idsl", [])
        self.assertCountEqual(parsing_utils.generate_recursive_imports(["AprilTags.idsl"], []),
                              ['JointMotor.idsl', 'GenericBase.idsl'])

    def test_communication_is_ice(self):
        self.assertTrue(parsing_utils.communication_is_ice("CameraSimple"))
        self.assertTrue(parsing_utils.communication_is_ice("Ca"))
        self.assertTrue(parsing_utils.communication_is_ice(["CameraSimple", "ice"]))
        self.assertFalse(parsing_utils.communication_is_ice(["CameraSimple", "ros"]))
        self.assertRaises(ValueError, parsing_utils.communication_is_ice, ["CameraSimple", "nus"])
        self.assertTrue(parsing_utils.communication_is_ice(["CameraSimple"]))

    def test_IDSLPool(self):
        pool = parsing_utils.IDSLPool("AprilTags.idsl", [])
        self.assertIn("AprilTags", pool.modulePool)
        self.assertIn("GenericBase", pool.modulePool)
        self.assertIn("JointMotor", pool.modulePool)

        module = pool.moduleProviding("JointMotor")
        self.assertEqual(module['name'], 'RoboCompJointMotor')

        idsl_module = pool.IDSLsModule(module)
        self.assertEqual(idsl_module, '/opt/robocomp/interfaces/IDSLs/JointMotor.idsl')

        interfaces = pool.interfaces()
        self.assertCountEqual(interfaces, ['GenericBase', 'JointMotor', 'JointMotorPublish', 'AprilTags'])

        ros_imports = pool.rosImports()
        self.assertCountEqual(ros_imports,
                              ['RoboCompGenericBaseROS/TBaseState', 'std_msgs/Int32', 'std_msgs/Float32',
                               'RoboCompJointMotorROS/MotorState', 'RoboCompJointMotorROS/MotorParams',
                               'RoboCompJointMotorROS/BusParams', 'RoboCompJointMotorROS/MotorGoalPosition',
                               'RoboCompJointMotorROS/MotorGoalVelocity', 'RoboCompJointMotorROS/MotorParamsList',
                               'RoboCompJointMotorROS/MotorGoalPositionList',
                               'RoboCompJointMotorROS/MotorGoalVelocityList', 'RoboCompJointMotorROS/MotorList',
                               'std_msgs/String', 'RoboCompAprilTagsROS/tag', 'RoboCompAprilTagsROS/tagsList'])

        ros_module_imports = pool.rosModulesImports()
        self.assertCountEqual(ros_module_imports, [{'strName': 'TBaseState', 'name': 'RoboCompGenericBase'},
                                                   {'strName': 'MotorState', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorParams', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'BusParams', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorGoalPosition', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorGoalVelocity', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorParamsList', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorGoalPositionList',
                                                    'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorGoalVelocityList',
                                                    'name': 'RoboCompJointMotor'},
                                                   {'strName': 'MotorList', 'name': 'RoboCompJointMotor'},
                                                   {'strName': 'tag', 'name': 'RoboCompAprilTags'},
                                                   {'strName': 'tagsList', 'name': 'RoboCompAprilTags'}])

    def test_is_agm1_agent(self):
        component = DSLFactory().from_file(
            os.path.join(CURRENT_DIR, "resources", "camerasimple.cdsl"))
        self.assertFalse(parsing_utils.is_agm1_agent(component))
        component = DSLFactory().from_file(
            os.path.join(CURRENT_DIR, "resources", "humanAgent.cdsl"))
        self.assertTrue(parsing_utils.is_agm1_agent(component))
        self.assertRaises(AssertionError, parsing_utils.is_agm1_agent, "CameraSimple")

    # def test_is_agm2_agent(self):
    #     self.assertRaises(AssertionError, parsing_utils.is_agm2_agent, "CameraSimple")
    #     # self.assertFalse(parsing_utils.is_agm2_agent(component))
    #     self.fail("There's no component (cdsl) available to test this method option")

    def test_idsl_robocomp_path(self):
        self.assertRaises(AssertionError, parsing_utils.idsl_robocomp_path, "CameraSimple.idsl", "any/valid/path")
        self.assertRaises(AssertionError, parsing_utils.idsl_robocomp_path, 12)
        self.assertTrue(parsing_utils.idsl_robocomp_path("CameraSimple.idsl") in
                        ["/opt/robocomp/interfaces/IDSLs/CameraSimple.idsl",
                         os.path.expanduser('~/robocomp/interfaces/IDSLs/CameraSimple.idsl')])

    def test_get_name_number(self):
        self.assertCountEqual(parsing_utils.get_name_number(['AGMExecutiveTopic', 'HumanPose']),
                              [['AGMExecutiveTopic', ''], ['HumanPose', '']])
        self.assertCountEqual(parsing_utils.get_name_number(['AGMExecutiveTopic', 'HumanPose', 'HumanPose']),
                              [['AGMExecutiveTopic', ''], ['HumanPose', ''], ['HumanPose', '1']])
        self.assertCountEqual(parsing_utils.get_name_number(['HumanPose', 'HumanPose', 'HumanPose']),
                              [['HumanPose', ''], ['HumanPose', '1'], ['HumanPose', '2']])
        self.assertRaises(AssertionError, parsing_utils.get_name_number, "lapatochada")
        self.assertRaises(AssertionError, parsing_utils.get_name_number, ["lapatochada", 8, 3.9])

    def test_decorator_and_type_to_const_ampersand(self):
        modulePool = parsing_utils.IDSLPool(["AprilTags.idsl"], [])
        # theInterface = theInterface.split(';')
        # module = modulePool.moduleProviding(theInterface[0])
        # if module == None:
        #     print('Can\'t locate', theIDSLs)
        #     sys.exit(1)
        self.fail()
