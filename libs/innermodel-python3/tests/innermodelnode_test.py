import unittest
import sys

sys.path.append('../innermodel_python3')

from innermodelnode import InnerModelNode

class TestInnerModelNode (unittest.TestCase):
    def test_constructor (self):
        root = InnerModelNode ('root', None)
        self.assertTrue (root is not None)

        child = InnerModelNode ('child1', parent=root)
        self.assertTrue (child is not None)

    def test_print (self):
        root = InnerModelNode ('root', None)
        child1 = InnerModelNode ('child1', parent=root)
        root.addChild (child1)
        child2 = InnerModelNode ('child2', None)
        root.addChild (child2)
        root.printTree ('', True)

    def test_setparent (self):
        node1 = InnerModelNode ('node1', None)
        node2 = InnerModelNode ('node2', None)

        node2.setParent (parent=node1)
        self.assertTrue (node2.parent.id == 'node1')

    def test_addchild (self):
        root = InnerModelNode ('root', None)
        child1 = InnerModelNode ('child1', parent=root)
        root.addChild (child1)
        child2 = InnerModelNode ('child2', parent=root)
        root.addChild (child2)

        self.assertTrue (len(root.children) == 2)
        self.assertTrue (child1 in root.children)
        self.assertTrue (child2 in root.children)

if __name__ == '__main__':
    unittest.main()
