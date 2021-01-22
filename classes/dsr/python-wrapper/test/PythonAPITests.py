import unittest
import subprocess

import sys, time, os
sys.path.append("/opt/robocomp/lib")
from pydsr import *


ETC_DIR = "../etc/"

class TestAttribute(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_attribute(self):
        tmp = Attribute(10.4, 0, 12)
        self.assertIsNotNone(tmp)
        self.assertAlmostEqual(tmp.value, 10.4, 5)
        with self.assertRaises(RuntimeError):
            tmp.value = True

    def test_update_value(self):
        tmp = Attribute(10.4, 0, 12)
        self.assertAlmostEqual(tmp.value, 10.4, 5)
        tmp.value = 0.0
        self.assertAlmostEqual(tmp.value, 0.0, 5)

    def test_agent_id(self):
        tmp = Attribute(10.4, 0, 12)
        self.assertEqual(tmp.agent_id, 12)
        with self.assertRaises(AttributeError):
            tmp.agent_id = 100

    def test_timestamp(self):
        tmp = Attribute(10.4, 0, 12)
        self.assertEqual(tmp.timestamp, 0)
        with self.assertRaises(AttributeError):
            tmp.timestamp = 100
    

class TestEdge(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_edge(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertIsNotNone(tmp)


    def test_type(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertEqual(tmp.type, "RT")
        with self.assertRaises(AttributeError):
            tmp.type = "NOTYPE"

    def test_from(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertEqual(tmp.origin, 11)
        with self.assertRaises(AttributeError):
            tmp.origin = 22

    def test_to(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertEqual(tmp.destination, 10)
        with self.assertRaises(AttributeError):
            tmp.destination = 22

    def test_agent_id(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertEqual(tmp.agent_id, 0)
        tmp.agent_id = 22
        self.assertEqual(tmp.agent_id, 22)

    def test_attrs(self):
        tmp = Edge(10,11, "RT", 0)
        self.assertEqual(len(tmp.attrs), 0)
        tmp.attrs["test"] = Attribute(10.4, 0, 12)
        self.assertEqual(len(tmp.attrs), 1)
        self.assertIsNotNone(tmp.attrs["test"])
        del tmp.attrs["test"]
        with self.assertRaises(KeyError):
            tmp.attrs["test"]



class TestNode(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_node(self):
        tmp = Node(1, "world")
        self.assertIsNotNone(tmp)
        with self.assertRaises(RuntimeError):
            tmp = Node(1, "test")
        
    def test_name(self):
        tmp = Node(1, "world", "name")
        self.assertEqual(tmp.name, "name")
        with self.assertRaises(AttributeError):
            tmp.name = "newname"

    def test_type(self):
        tmp = Node(1, "world","name")
        self.assertEqual(tmp.type, "world")
        with self.assertRaises(AttributeError):
            tmp.id = "newtype"

    def test_agent_id(self):
        tmp = Node(1, "world", "name")
        self.assertEqual(tmp.agent_id, 1)
        tmp.agent_id = 2
        self.assertEqual(tmp.agent_id, 2)

    def test_attrs(self):
        tmp = Node(1, "world", "name")
        self.assertEqual(len(tmp.attrs), 0)
        tmp.attrs["test"] = Attribute(10.4, 0, 12)
        self.assertEqual(len(tmp.attrs), 1)
        self.assertIsNotNone(tmp.attrs["test"])
        del tmp.attrs["test"]
        with self.assertRaises(KeyError):
            tmp.attrs["test"]

    def test_edge(self):
        tmp = Node(1, "world", "name")
        self.assertEqual(len(tmp.edges), 0)
        tmp.edges[(11,"RT")] = Edge(10,11, "RT", 0)
        self.assertEqual(len(tmp.edges), 1)
        self.assertIsNotNone(tmp.edges[(11,"RT")])
        del tmp.edges[(11,"RT")]
        with self.assertRaises(KeyError):
            tmp.edges[(11,"RT")]



class TestDSRGraph(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_graph(self):
        a = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        self.assertIsNotNone(a)

    def test_get_node(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        node = g.get_node(1)
        self.assertIsNotNone(node)
        node = g.get_node("world")
        self.assertIsNotNone(node)
        node = g.get_node(1000000)
        self.assertIsNone(node)
        node = g.get_node("weirdname")
        self.assertIsNone(node)

    def test_delete_node(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        node = g.get_node(1)
        self.assertIsNotNone(node)
        res = g.delete_node(1)
        self.assertEqual(res, True)
        node = g.get_node(1)
        self.assertIsNone(node)

    
    def test_update_node(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        world = g.get_node("world")
        world.attrs["color"].value = "red"
        result = g.update_node(world)
        self.assertEqual(result, True)
        world = g.get_node("world")
        self.assertEqual(world.attrs["color"].value, "red")
        

    def test_get_edge(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)

        edge = g.get_edge(1, 2, "RT")
        self.assertIsNotNone(edge)
        edge = g.get_edge(g.get_name_from_id(1), g.get_name_from_id(2), "RT")
        self.assertIsNotNone(edge)
        edge = g.get_edge(11111, 22222, "RT")
        self.assertIsNone(edge)
        edge = g.get_edge("11111", "22222", "RT")
        self.assertIsNone(edge)

        
    def test_insert_or_asssign_edge(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        edge = g.get_edge(1, 2, "RT")
        edge.attrs["color"] = Attribute("red", 0, 12)
        g.insert_or_assign_edge(edge)
        edge = g.get_edge(1, 2, "RT")
        self.assertEqual(edge.attrs["color"].value, "red")
        edge = Edge(1, 2, "in", 12)
        g.insert_or_assign_edge(edge)
        edge = g.get_edge(2, 1, "in")
        self.assertIsNotNone(edge)


    def test_delete_edge(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        edge = g.get_edge(1, 2, "RT")
        self.assertIsNotNone(edge)
        result = g.delete_edge(1, 2, "RT")
        self.assertEqual(result, True)
        edge = g.get_edge(1, 2, "RT")
        self.assertIsNone(edge)
        result = g.delete_edge(111111, 22222, "RT")
        self.assertEqual(result, False)

    def test_get_node_root(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)

        node = g.get_node_root()
        self.assertIsNotNone(node)
        self.assertEqual(node.id, 1)
        self.assertEqual(node.name, "world")

    def test_get_nodes_by_type(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        nodes = g.get_nodes_by_type("rgbd")
        self.assertEqual(len(nodes), 1)
        nodes = g.get_nodes_by_type("invalidtype")
        self.assertEqual(len(nodes), 0)

    def test_get_name_from_id(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        name = g.get_name_from_id(1)
        self.assertIsNotNone(name)
        self.assertEqual(name, "world")
        name = g.get_name_from_id(11111111)
        self.assertIsNone(name)

    def test_get_id_from_name(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        id = g.get_id_from_name("world")
        self.assertIsNotNone(id)
        self.assertEqual(id, 1)
        id = g.get_id_from_name("11111111")
        self.assertIsNone(id)

    def test_get_edges_by_type(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        edges = g.get_edges_by_type("RT")
        self.assertGreater(len(edges), 0)
        edges = g.get_edges_by_type("invalidtype")
        self.assertEqual(len(edges), 0)
        

    def test_get_edges_to_id(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        edges = g.get_edges_to_id(2)
        self.assertGreater(len(edges), 0)
        edges = g.get_edges_to_id(1)
        self.assertEqual(len(edges), 0)


    def test_insert_node(self):

        g = DSRGraph(int(0), "Prueba", 12, os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json") )
        node = Node(12, "mesh", "newmesh")

        id = g.insert_node(node)
        self.assertIsNotNone(id)
        node = g.get_node("newmesh")
        self.assertIsNotNone(node)
        self.assertEqual(id, node.id)


    

class TestRTAPI(unittest.TestCase):
    

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_rtapi(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        self.assertIsNotNone(rt_api)
    
    def test_insert_or_assign_edge_RT(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        world = g.get_node("world")
        rt.insert_or_assign_edge_RT(world, 203, [0.0, 1.2, 0.0], [1.1, 0.0, 2.2])
        world = g.get_node("world")
        self.assertIsNotNone(world.edges[(203, "RT")])

    def test_get_edge_RT(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        edge = rt.get_edge_RT(g.get_node("world"),2)
        self.assertIsNotNone(edge)
        self.assertEqual(edge.type, "RT")


    def test_get_RT_pose_from_parent(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        pose = rt.get_RT_pose_from_parent(g.get_node(2))
        self.assertIsNotNone(pose)


    def test_get_edge_RT_as_rtmat(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        edge = rt.get_edge_RT(g.get_node("world"),2)
        rtmat = rt.get_edge_RT_as_rtmat(edge)
        self.assertIsNotNone(rtmat)

    def test_get_translation(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        rt = rt_api(g)
        trans = rt.get_translation(1, 2)
        self.assertIsNotNone(trans)

class TestInnerAPI(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        #time.sleep(0.5)
        pass

    def test_create_innerapi(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        self.assertIsNotNone(inner)

    def test_transform(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        tr = inner.transform("world", "laser")
        self.assertIsNotNone(tr)
        tr = inner.transform("world", [1.1, 3.3, 6.6], "laser")
        self.assertIsNotNone(tr)

    def test_transform_axis(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        tr = inner.transform_axis("world", "laser")
        self.assertIsNotNone(tr)
        tr = inner.transform_axis("world", [1.1, 3.3, 6.6, 0.0 , 0.0, 0.0], "laser")
        self.assertIsNotNone(tr)

    def test_get_transformation_matrix(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        tr_matrix = inner.transform_axis("world", "laser")
        self.assertIsNotNone(tr_matrix)

    def test_get_rotation_matrix(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        rot = inner.get_rotation_matrix("world", "laser")
        self.assertIsNotNone(rot)

    def test_get_translation_vector(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        trans = inner.get_translation_vector("world", "laser")
        self.assertIsNotNone(trans)

    def test_get_euler_xyz_angles(self):
        g = DSRGraph(int(0), "Prueba", int(12), os.path.join(ETC_DIR, "autonomyLab_complete.simscene.json"), True)
        inner = inner_api(g)
        angles = inner.get_euler_xyz_angles("world", "laser")
        self.assertIsNotNone(angles)


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]



if __name__ == '__main__':
    #import psutil
    #if psutil.Process(os.getpid()).parent().name() == 'sh':
    #    unittest.main()
    #else:
    #    print("You probably want to execute test with the runTest.sh script.")
    unittest.main()