import unittest
from unittest import mock
import sys
import os

from typer.testing import CliRunner

from rcworkspace.main import app
from rcworkspace.workspace import Workspace

sys.path.append(os.path.join(os.path.dirname(__file__), "../rcworkspace"))

class TestRCWorkspace(unittest.TestCase):

    def test_add_workspace(self):
        with mock.patch('os.walk') as mockwalk:
            mockwalk.return_value = [
                ('/foo', ('bar',), ('baz',)),
                ('/foo/bar', (), ('spam', 'eggs')),
            ]
            ws = Workspace()
            ws.add_workspace("sadasd", True)




# runner = CliRunner()
#
#
#
#
# class TestRCWorkspace(unittest.TestCase):
#
#     def test_app(self):
#         result = runner.invoke(app, ["--help", ])
#         self.assertTrue(result.exit_code == 0)
#         result = runner.invoke(app, ["add"])
#         print(result.stdout)

if __name__ == '__main__':
    unittest.main()

