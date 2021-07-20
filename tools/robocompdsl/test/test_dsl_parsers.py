import json
import os
import sys
import unittest
from collections import OrderedDict
from pyparsing import ParseSyntaxException

from pyparsing import ParseException
import copy

sys.path.append("/opt/robocomp/python")
sys.path.append('/opt/robocomp/python/robocompdsl/')
import dsl_parsers.specific_parsers.cdsl.componentfacade as cf

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
RESOURCES_DIR = os.path.join(CURRENT_DIR, "resources")
sys.path.append(ROBOCOMPDSL_DIR)

from dsl_parsers.specific_parsers.cdsl.cdsl_parser import CDSLParser


class CDSLParserTestCase(unittest.TestCase):

    def test_init(self):
        cdsl_parser = CDSLParser()
        self.assertEqual(cdsl_parser._include_directories, [])

        paths = ['/one/path', '/other,path']
        cdsl_parser = CDSLParser(paths)
        self.assertEqual(cdsl_parser._include_directories, paths)

        self.assertRaises(AssertionError, CDSLParser, "one_path")

    def test_print(self):
        cdsl_parser = CDSLParser()
        self.assertEqual(str(cdsl_parser), '<empty>')
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
        cdsl_parser = CDSLParser()
        cdsl_parser.string_to_struct(valid_cdsl_string)
        self.assertEqual(str(cdsl_parser),
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

    def test_string_to_struct(self):
        parser = CDSLParser()
        self.assertRaises(AssertionError, parser.string_to_struct, "")
        self.assertRaises(AssertionError, parser.string_to_struct, [])
        self.assertRaises(ParseSyntaxException, parser.string_to_struct, "NotValidCDSL string")
        # TODO: Test valid cdsls that completes de coverage
        valid_cdsl_string = """      
Component TheComponentName
{
        Communications
        {

        };
        language Python;
        options dsr;
};
"""
        component = parser.string_to_struct(valid_cdsl_string, include_directories=['/one/dir/'])
        self.assertEqual(component.language, "python")
        self.assertEqual(component.name, "TheComponentName")
        self.assertIn("dsr", component.options)

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
        self.assertRaises(ParseSyntaxException, parser.string_to_struct, invalid_option_cdsl_string)

        valid_agm_cdsl_string = """      
        Component TheComponentName
        {
                Communications
                {

                };
                language Python;
                options agmagent;
        };
        """
        component = parser.string_to_struct(valid_agm_cdsl_string, include_directories=['/one/dir/'])


if __name__ == '__main__':
    unittest.main()
