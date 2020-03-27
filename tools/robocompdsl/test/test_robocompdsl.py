import os
import sys
import unittest

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
TEST_DIR = os.path.join(CURRENT_DIR, "..", "autogeneration_tests", "test_cdsl")
sys.path.append(ROBOCOMPDSL_DIR)

from autogeneration_tests.test_cdsl.test_component_generation import ComponentGenerationChecker


class ParsingUtilsTest(unittest.TestCase):
    def test_component_generation_checker(self):
        checker = ComponentGenerationChecker()
        self.assertTrue(checker.check_components_generation(TEST_DIR, False, False, ignore="test_agmTestCpp"))


# component = specific_parsers.DSLFactory().from_file("/home/robolab/robocomp/components/robocomp-robolab/components/hardware/camera/camerasimple/camerasimple.cdsl")
# robocompdsl.generate_ROS_headers("CameraSimple.idsl", ".", component, ["/home/robolab/robocomp/interfaces/IDSLs/"])
