import sys
import os
import unittest
from pyparsing import ParseSyntaxException, ParseException
import config_tests
from robocompdsl.dsl_parsers.specific_parsers.cdsl.cdsl_parser import CDSLParser

class CDSLParserTestCase(unittest.TestCase):

    def setUp(self) -> None:
        self.cdsl_parser = CDSLParser()

    def test_init(self):
        self.assertEqual(CDSLParser().include_directories, [])


    def test_init_with_paths(self):
        # Valid paths input
        paths = ['/one/path', '/other,path']
        cdsl_parser = CDSLParser(paths)
        self.assertEqual(cdsl_parser.include_directories, paths)
        # Not valid paths input
        self.assertRaises(AssertionError, CDSLParser, "one_path")

    def test_print(self):
        self.assertEqual(str(self.cdsl_parser), '<empty>')
        valid_cdsl_string = """
        import "CameraSimple.idsl";     
        Component TheComponentName
        {
                Communications
                {

                };
                language Python;
                options dsr;
        };
        """
        self.cdsl_parser.string_to_struct(valid_cdsl_string)
        self.assertEqual(str(self.cdsl_parser),
"""Component: TheComponentName
	Imports:
		 CameraSimple.idsl
	Language:		 python
	GUI:
		 None
	Communications:
		Implements [] 
		Requires []
		Publishes []
		Subscribes []
""")

    def test_string_to_struct_invalid_inputs(self):

        self.assertRaises(AssertionError, self.cdsl_parser.string_to_struct, "")
        self.assertRaises(AssertionError, self.cdsl_parser.string_to_struct, [])
        self.assertRaises(ParseSyntaxException, self.cdsl_parser.string_to_struct, "NotValidCDSL string")

        invalid_option_cdsl_string = """      
               Component TheComponentName
               {
                       Communications
                       {

                       };
                       language Python;
                       options not_valid_option;
               };
               """
        self.assertRaises(ParseSyntaxException, self.cdsl_parser.string_to_struct, invalid_option_cdsl_string)

    def test_string_to_struct_valid_inputs(self):
        valid_cdsl_string = """      
Component TheComponentName
{
        Communications
        {

        };
        language Python;
        gui Qt(QWidget);
        options dsr;
        statemachine "statemachine.smdsl" visual;

};
"""
        component = self.cdsl_parser.string_to_struct(valid_cdsl_string, include_directories=['/one/dir/'])
        self.assertEqual(component.language, "python")
        self.assertEqual(component.name, "TheComponentName")
        self.assertEqual(component.statemachine_path, "statemachine.smdsl")
        self.assertTrue(component.statemachine_visual)
        self.assertTrue(component.dsr)
        self.assertFalse(component.usingROS)
        self.assertIn("dsr", component.options)
        self.assertListEqual(component.gui, ['Qt', 'QWidget'])

    def test_string_to_struct_valid_inputs_agm(self):

        valid_agm_cdsl_string = """      
        Component TheComponentName
        {
                Communications
                {
                    implements CameraSimple;
                };
                language Python;
                options agmagent;
        };
        """
        component = self.cdsl_parser.string_to_struct(valid_agm_cdsl_string)

        self.assertTrue(component.is_agm_agent())
        self.assertIn(['CameraSimple', 'ice'], component.implements)

    def test_string_to_struct_valid_inputs_ros_comm(self):

        valid_cdsl_ros_string = """      
        Component TheComponentName
        {
                Communications
                {
                    implements CameraSimple(ros);
                };
                language Python;
        };
        """
        component = self.cdsl_parser.string_to_struct(valid_cdsl_ros_string)

        self.assertTrue(component.usingROS)
        self.assertListEqual(component.rosInterfaces, [['CameraSimple', 'ros']])
        self.assertListEqual(component.implements, [['CameraSimple', 'ros']])

if __name__ == '__main__':
    unittest.main()
