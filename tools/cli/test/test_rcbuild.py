import unittest

from typer.testing import CliRunner

from rcbuild.rcbuild import app

runner = CliRunner()


class TestRCBuild(unittest.TestCase):

    def test_app(self):
        result = runner.invoke(app, ["--help", ])
        self.assertTrue(result.exit_code == 0)
        result = runner.invoke(app, ["comp", "carla"])
        self.assertTrue(result.exit_code == 2)

if __name__ == '__main__':
    unittest.main()

