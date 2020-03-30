import os
import random
import sys
import tempfile
import unittest
import robocompdsl

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
RESOURCES_DIR = os.path.join(CURRENT_DIR, "resources")
TEST_DIR = os.path.join(CURRENT_DIR, "..", "autogeneration_tests", "test_cdsl")
REF_COMPONENTS_PATH = os.path.join(RESOURCES_DIR, "reference_components")
sys.path.append(ROBOCOMPDSL_DIR)

from autogeneration_tests.test_cdsl.test_component_generation import ComponentGenerationChecker


class ParsingUtilsTest(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None
        self.tempdir = os.path.join(tempfile.gettempdir(), 'testcog_tempdir_' + str(random.random())[2:])

    def test_component_generation_checker(self):
        checker = ComponentGenerationChecker()
        self.assertTrue(checker.check_components_generation(TEST_DIR, False, False, ignore="test_agmTestCpp"))

    def test_idsl_creation(self):
        input_file = os.path.join(RESOURCES_DIR, "InnerModelManager.idsl")
        output_file = os.path.join(RESOURCES_DIR, "InnerModelManager_generated.ice")
        truth_file = os.path.join(RESOURCES_DIR, "InnerModelManager.ice")
        robocompdsl.generate_idsl_file(input_file, output_file, [])
        self.assertFilesSame(output_file, truth_file)
        os.remove(output_file)


    # def test_python_component_creation(self):
    #     python_components = [
    #         "test_customStateMachinePython",
    #         "test_defaultStateMachinePython",
    #         "test_noSmdslTestPython",
    #         "test_subStatesTestPython"
    #     ]
    #     python_cdsls_path = [os.path.join(REF_COMPONENTS_PATH, component, "test.cdsl") for component in
    #                          python_components]
    #     for cdsl in python_cdsls_path:
    #         robocompdsl.generate_component_from_cdsl(cdsl, self.tempdir, [])



    def assertFilesSame(self, path1, path2):
        with open(path1, 'r') as f1, open(path2, 'r') as f2:
            text1 = f1.readlines()
            text2 = f2.readlines()
        self.assertEqual(text1, text2)



# component = specific_parsers.DSLFactory().from_file("/home/robolab/robocomp/components/robocomp-robolab/components/hardware/camera/camerasimple/camerasimple.cdsl")
# robocompdsl.generate_ROS_headers("CameraSimple.idsl", ".", component, ["/home/robolab/robocomp/interfaces/IDSLs/"])
