import os
import random
import shutil
import tempfile
import unittest

from config_tests import CURRENT_DIR
from robocompdsl.common.filesgenerator import FilesGenerator

RESOURCES_DIR = os.path.join(CURRENT_DIR, "resources")
TEST_DIR = os.path.join(CURRENT_DIR, "..", "autogeneration_tests", "test_cdsl")
REF_COMPONENTS_PATH = os.path.join(RESOURCES_DIR, "reference_components")


class RobocompdslTest(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def renew_temp_dir(self, name=None):
        if name is None:
            self.tempdir = os.path.join(tempfile.gettempdir(), 'testrobocompdsl_tempdir_' + str(random.random())[2:])
        else:
            self.tempdir = os.path.join(tempfile.gettempdir(), name)
        shutil.rmtree(self.tempdir, ignore_errors=True)
        os.mkdir(self.tempdir)

    # def test_component_generation_checker(self):
    #     checker = ComponentGenerationChecker()
    #     self.assertTrue(checker.check_components_generation(TEST_DIR, False, False, ignore="test_agmTestCpp"))

    def test_idsl_creation(self):
        input_file = os.path.join(RESOURCES_DIR, "InnerModelManager.idsl")
        output_file = os.path.join(RESOURCES_DIR, "InnerModelManager_generated.ice")
        truth_file = os.path.join(RESOURCES_DIR, "InnerModelManager.ice")
        try:
            os.remove(output_file)
        except FileNotFoundError:
            pass
        FilesGenerator().generate(input_file, output_file, [])
        self.assertFilesSame(truth_file, output_file)

        # self.assertRaises(FileNotFoundError,robocompdsl.generate_idsl_file("NonExisting.idsl", "outputfile.ice", []))
        # os.remove("outputfile.ice")


    def test_python_components_creation(self):
        testing_components = testing_components = [
                "test_allCommunicationsPython",
                "test_customStateMachinePython",
                "test_defaultStateMachinePython",
                "test_implementsPython",
                "test_noSmdslTestPython",
                "test_publicationPython",
                "test_subscriptionPython",
                "test_subStatesTestPython",
                "test_dsr2Python"
        ]
        self.component_creation(testing_components)

    def test_cpp_components_creation(self):
        testing_components = [
                "test_agmTestCpp",
                "test_allCommunicationsCpp",
                "test_Cpp11",
                "test_customStateMachineCpp",
                "test_defaultStateMachineCpp",
                "test_noSmdslTestCpp",
                "test_publicationCpp",
                "test_subscriptionCpp",
                "test_subStatesTestCpp",
                "test_dsr2Cpp"
        ]
        self.component_creation(testing_components)

    def component_creation(self, component_list):
        for current_component in component_list:
            component_path = os.path.join(REF_COMPONENTS_PATH, current_component)
            self.renew_temp_dir(current_component)
            cdsl = os.path.join(component_path, 'testcomp.cdsl')
            cdsl = shutil.copy(cdsl, self.tempdir)
            smdsl = os.path.join(component_path, 'statemachine.smdsl')
            if os.path.exists(smdsl):
                smdsl = shutil.copy(smdsl, self.tempdir)
            self.olddir = os.getcwd()
            os.chdir(self.tempdir)
            try:
                FilesGenerator().generate(cdsl, self.tempdir, [])
            except Exception as e:
                self.fail(str(e))
            else:
                self.compare_components(component_path, self.tempdir)
            finally:
                os.chdir(self.olddir)
                shutil.rmtree(self.tempdir, ignore_errors=True)

    # def test_invalid_language(self):
    #     self.renew_temp_dir("Invalid")
    #     cdsl = os.path.join(RESOURCES_DIR, "InvalidLanguage.cdsl")
    #     # self.assertRaises(ValueError, FilesGenerator().generate, cdsl, self.tempdir, [])
    #     shutil.rmtree(self.tempdir, ignore_errors=True)


    def assertFilesSame(self, path1, path2):
        print("Checking file %s" % os.path.basename(path1))
        with open(path1, 'r', encoding='utf-8', errors='ignore') as f1, open(path2, 'r', encoding='utf-8', errors='ignore') as f2:
            text1 = f1.readlines()
            text2 = f2.readlines()
            import re
            # Remove files with copyright from the comparation
            text1 = [line for line in text1 if not re.search('Copyright \(C\)', line)]
            text2 = [line for line in text2 if not re.search('Copyright \(C\)', line)]
            # Remove lines with configured ports from the comparation
            if path1.endswith("config") and path2.endswith("config"):
                text1 = [line for line in text1 if not re.search(r'-p \d{3}', line)]
                text2 = [line for line in text2 if not re.search(r'-p \d{3}', line)]
        self.assertEqual(text1, text2)

    def compare_components(self, reference, generated):
        for root, dirnames, filenames in os.walk(reference):
            ref_root = root.replace(reference, generated)
            generated_files = os.listdir(ref_root)
            for filename in filenames:
                with self.subTest("Component comparation for %s" % root, directory=root, filename=filename):
                    if filename in generated_files:
                        self.assertFilesSame(
                            os.path.join(root, filename),
                            os.path.join(ref_root, filename)
                        )
                    else:
                        self.fail("File %s found in reference is not in generated comp" % filename)
