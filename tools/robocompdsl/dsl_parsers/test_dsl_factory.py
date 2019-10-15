import unittest

from dsl_parsers.dsl_factory import DSLFactory


class DSLFactoryTestCase(unittest.TestCase):

    def test_dsl_factory(self):
        factory = DSLFactory()
        factory2 = DSLFactory()
        # test of singleton factory behavior
        self.assertIs(factory, factory2)

        a = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/gamestatemachine.smdsl")
        # test for cached query
        b = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/gamestatemachine.smdsl")
        self.assertIs(a, b)

        c = factory.from_file("/home/robolab/robocomp/interfaces/IDSLs/JointMotor.idsl")
        # test for cached query
        d = factory.from_file("/home/robolab/robocomp/interfaces/IDSLs/JointMotor.idsl")
        self.assertIs(c, d)

        e = factory.from_file("/home/robolab/robocomp/tools/robocompdsl/component_generation_test/customStateMachinePython/test.cdsl")
        # test for cached query
        f = factory.from_file("/home/robolab/robocomp/tools/robocompdsl/component_generation_test/customStateMachinePython/test.cdsl")
        self.assertIs(e, f)




if __name__ == '__main__':
    unittest.main()
