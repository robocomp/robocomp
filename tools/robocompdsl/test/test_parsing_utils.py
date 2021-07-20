import os
import sys
import unittest
from unittest import TestCase

from dsl_parsers.specific_parsers.cdsl.componentfacade import Interface
from dsl_parsers import parsing_utils
from dsl_parsers.dsl_factory import DSLFactory

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..")
RESOURCES_DIR = os.path.join(CURRENT_DIR, "resources")
sys.path.append(ROBOCOMPDSL_DIR)


# noinspection PyCompatibility
class ParsingUtilsTest(unittest.TestCase):

    def test_generate_recursive_imports(self):
        # input string but list is expected
        self.assertRaises(AssertionError, parsing_utils.generate_recursive_imports, "AprilTags.idsl", [])

        self.assertRaises(FileNotFoundError, parsing_utils.generate_recursive_imports, ["NoExists.idsl"], [])

        self.assertCountEqual(parsing_utils.generate_recursive_imports(["AprilTags.idsl"], []),
                              ['JointMotor.idsl', 'GenericBase.idsl'])

    def test_communication_is_ice(self):
        self.assertTrue(parsing_utils.communication_is_ice("CameraSimple"))
        self.assertTrue(parsing_utils.communication_is_ice("Ca"))
        self.assertTrue(parsing_utils.communication_is_ice(["CameraSimple", "ice"]))
        self.assertFalse(parsing_utils.communication_is_ice(["CameraSimple", "ros"]))
        self.assertRaises(ValueError, parsing_utils.communication_is_ice, ["CameraSimple", "nus"])
        self.assertRaises(ValueError, parsing_utils.communication_is_ice, 12)
        self.assertTrue(parsing_utils.communication_is_ice(["CameraSimple"]))

    def test_IDSLPool(self):
        self.assertRaises(AssertionError, parsing_utils.IDSLPool, "AprilTags.idsl", [])
        pool = parsing_utils.IDSLPool(["AprilTags.idsl"], [])
        self.assertIn("AprilTags", pool)
        self.assertIn("GenericBase", pool)
        self.assertIn("JointMotor", pool)

        module = pool.module_providing_interface("JointMotor")
        self.assertEqual(module['name'], 'RoboCompJointMotor')

        idsl_module = pool.IDSL_file_for_module(module)
        self.assertEqual(idsl_module, '/opt/robocomp/interfaces/IDSLs/JointMotor.idsl')

        interfaces = pool.interfaces()
        self.assertCountEqual(interfaces, ['GenericBase', 'JointMotor', 'JointMotorPublish', 'AprilTags', 'CommonBehavior'])

    def test_is_agm_agent(self):
        component = DSLFactory().from_file(
            os.path.join(CURRENT_DIR, "resources", "camerasimple.cdsl"))
        self.assertFalse(component.is_agm_agent())
        component = DSLFactory().from_file(
            os.path.join(CURRENT_DIR, "resources", "humanAgent.cdsl"))
        self.assertTrue(component.is_agm_agent())

    # def test_is_agm2_agent(self):
    #     self.assertRaises(AssertionError, parsing_utils.is_agm2_agent, "CameraSimple")
    #     # self.assertFalse(component.is_agm2_agent())
    #     self.fail("There's no component (cdsl) available to test this method option")

    def test_idsl_robocomp_path(self):
        self.assertRaises(AssertionError, parsing_utils.idsl_robocomp_path, "CameraSimple.idsl", "any/valid/path")
        self.assertRaises(AssertionError, parsing_utils.idsl_robocomp_path, 12)
        self.assertTrue(parsing_utils.idsl_robocomp_path("CameraSimple.idsl") in
                        ["/opt/robocomp/interfaces/IDSLs/CameraSimple.idsl",
                         os.path.expanduser('~/robocomp/interfaces/IDSLs/CameraSimple.idsl')])

    def test_get_name_number(self):
        self.assertCountEqual(parsing_utils.get_name_number([Interface(['AGMExecutiveTopic', 'ice']), Interface(['HumanPose', 'ice'])]),
                              [[('AGMExecutiveTopic', 'ice'), ''], [('HumanPose', 'ice'), '']])
        self.assertCountEqual(parsing_utils.get_name_number([Interface(['AGMExecutiveTopic', 'ice']), Interface(['HumanPose', 'ice']), Interface(['HumanPose', 'ice'])]),
                              [[('AGMExecutiveTopic', 'ice'), ''], [('HumanPose', 'ice'), ''], [('HumanPose', 'ice'), '1']])
        self.assertCountEqual(parsing_utils.get_name_number([Interface(['HumanPose', 'ice']), Interface(['HumanPose', 'ice']), Interface(['HumanPose', 'ice'])]),
                              [[('HumanPose', 'ice'), ''], [('HumanPose', 'ice'), '1'], [('HumanPose', 'ice'), '2']])
        self.assertRaises(AssertionError, parsing_utils.get_name_number, "lapatochada")
        self.assertRaises(AssertionError, parsing_utils.get_name_number, ["lapatochada", 8, 3.9])

    def test_decorator_and_type_to_const_ampersand(self):
        type1 = ['float', 'int', 'short', 'long', 'double']
        for vtype in type1:
            self.assertEqual(
                parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype=vtype, module_pool=None, cpp11=False),
                ('const ', ' '))
            self.assertEqual(
                parsing_utils.decorator_and_type_to_const_ampersand(decorator='out', vtype=vtype, module_pool=None, cpp11=False),
                (' ', ' &'))

        # bool
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator='out', vtype='bool', module_pool=None,
                                                                cpp11=False),
            (' ', ' &'))
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype='bool', module_pool=None,
                                                                cpp11=False),
            (' ', ' '))

        # string
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator='out', vtype='string', module_pool=None,
                                                                cpp11=False),
            (' ', ' &'))
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype='string', module_pool=None,
                                                                cpp11=False),
            ('const ', ' &'))

        # custom types
        the_idsls = parsing_utils.generate_recursive_imports(["AprilTags.idsl"], [])
        the_idsls.append('AprilTags.idsl')
        module_pool = parsing_utils.IDSLPool(the_idsls, [])
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator='out', vtype='MotorParams', module_pool=module_pool,
                                                                cpp11=False),
            (' ', ' &'))
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype='MotorParams',
                                                                module_pool=module_pool,
                                                                cpp11=False),
            ('const ', ' &'))

        # cpp11 True
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype='MotorParams',
                                                                module_pool=module_pool,
                                                                cpp11=True),
            ('', ''))

        # invalid vtype
        self.assertRaises(TypeError,
            parsing_utils.decorator_and_type_to_const_ampersand, decorator=' ', vtype='InvalidType',
                                                                module_pool=module_pool,
                                                                cpp11=True)

        the_idsls = parsing_utils.generate_recursive_imports(["TouchPoints.idsl"], [])
        the_idsls.append('TouchPoints.idsl')
        module_pool = parsing_utils.IDSLPool(the_idsls, [])
        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator='out', vtype='StateEnum',
                                                                module_pool=module_pool,
                                                                cpp11=False),
            (' ', ' &'))

        self.assertEqual(
            parsing_utils.decorator_and_type_to_const_ampersand(decorator=' ', vtype='StateEnum',
                                                                module_pool=module_pool,
                                                                cpp11=False),
            (' ', ' '))


    # def test_gimme_idsl(self):
    #     # Full existing path
    #     a = parsing_utils.gimmeIDSL(os.path.join(RESOURCES_DIR, "InnerModelManager.idsl"))
    #     self.assertDictEqual({'name': 'RoboCompInnerModelManager', 'imports': '', 'recursive_imports': '', 'interfaces': [{'name': 'InnerModelManager', 'methods': OrderedDict([('addAttribute', {'name': 'addAttribute', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'idNode'}, {'decorator': 'none', 'type': 'string', 'name': 'name'}, {'decorator': 'none', 'type': 'string', 'name': 'type'}, {'decorator': 'none', 'type': 'string', 'name': 'value'}], 'throws': ['InnerModelManagerError']}), ('addJoint', {'name': 'addJoint', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'jointType', 'name': 'j'}], 'throws': ['InnerModelManagerError']}), ('addMesh', {'name': 'addMesh', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'meshType', 'name': 'm'}], 'throws': ['InnerModelManagerError']}), ('addPlane', {'name': 'addPlane', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'Plane3D', 'name': 'plane'}], 'throws': ['InnerModelManagerError']}), ('addTransform', {'name': 'addTransform', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'string', 'name': 'engine'}, {'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'Pose3D', 'name': 'pose'}], 'throws': ['InnerModelManagerError']}), ('collide', {'name': 'collide', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'a'}, {'decorator': 'none', 'type': 'string', 'name': 'b'}], 'throws': 'nothing'}), ('getAllNodeInformation', {'name': 'getAllNodeInformation', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'out', 'type': 'NodeInformationSequence', 'name': 'nodesInfo'}], 'throws': ['InnerModelManagerError']}), ('getAttribute', {'name': 'getAttribute', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'idNode'}, {'decorator': 'none', 'type': 'string', 'name': 'name'}, {'decorator': 'out', 'type': 'string', 'name': 'type'}, {'decorator': 'out', 'type': 'string', 'name': 'value'}], 'throws': ['InnerModelManagerError']}), ('getPose', {'name': 'getPose', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'out', 'type': 'Pose3D', 'name': 'pose'}], 'throws': ['InnerModelManagerError']}), ('getPoseFromParent', {'name': 'getPoseFromParent', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'out', 'type': 'Pose3D', 'name': 'pose'}], 'throws': ['InnerModelManagerError']}), ('getTransformationMatrix', {'name': 'getTransformationMatrix', 'decorator': '', 'return': 'Matrix', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'string', 'name': 'item'}], 'throws': ['InnerModelManagerError']}), ('moveNode', {'name': 'moveNode', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'src'}, {'decorator': 'none', 'type': 'string', 'name': 'dst'}], 'throws': ['InnerModelManagerError']}), ('removeAttribute', {'name': 'removeAttribute', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'idNode'}, {'decorator': 'none', 'type': 'string', 'name': 'name'}], 'throws': ['InnerModelManagerError']}), ('removeNode', {'name': 'removeNode', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}], 'throws': ['InnerModelManagerError']}), ('setAttribute', {'name': 'setAttribute', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'idNode'}, {'decorator': 'none', 'type': 'string', 'name': 'name'}, {'decorator': 'none', 'type': 'string', 'name': 'type'}, {'decorator': 'none', 'type': 'string', 'name': 'value'}], 'throws': ['InnerModelManagerError']}), ('setPlane', {'name': 'setPlane', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'Plane3D', 'name': 'plane'}], 'throws': ['InnerModelManagerError']}), ('setPointCloudData', {'name': 'setPointCloudData', 'decorator': '', 'return': 'void', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'id'}, {'decorator': 'none', 'type': 'PointCloudVector', 'name': 'cloud'}], 'throws': 'nothing'}), ('setPose', {'name': 'setPose', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'Pose3D', 'name': 'pose'}], 'throws': ['InnerModelManagerError']}), ('setPoseFromParent', {'name': 'setPoseFromParent', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'Pose3D', 'name': 'pose'}], 'throws': ['InnerModelManagerError']}), ('setScale', {'name': 'setScale', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'float', 'name': 'scaleX'}, {'decorator': 'none', 'type': 'float', 'name': 'scaleY'}, {'decorator': 'none', 'type': 'float', 'name': 'scaleZ'}], 'throws': ['InnerModelManagerError']}), ('transform', {'name': 'transform', 'decorator': '', 'return': 'bool', 'params': [{'decorator': 'none', 'type': 'string', 'name': 'base'}, {'decorator': 'none', 'type': 'string', 'name': 'item'}, {'decorator': 'none', 'type': 'coord3D', 'name': 'coordInItem'}, {'decorator': 'out', 'type': 'coord3D', 'name': 'coordInBase'}], 'throws': ['InnerModelManagerError']})])}], 'types': [{'type': 'enum', 'name': 'NodeType', 'content': ' Transform, Joint, DifferentialRobot, OmniRobot, Plane, Camera, RGBD, IMU, Laser, Mesh, PointCloud, TouchSensor, DisplayII '}, {'type': 'struct', 'name': 'AttributeType', 'structIdentifiers': [{'type': 'string', 'identifier': 'type'}, {'type': 'string', 'identifier': 'value'}]}, {'type': 'dictionary', 'content': 'string, AttributeType', 'name': 'AttributeMap'}, {'type': 'struct', 'name': 'NodeInformation', 'structIdentifiers': [{'type': 'string', 'identifier': 'id'}, {'type': 'string', 'identifier': 'parentId'}, {'type': 'NodeType', 'identifier': 'nType'}, {'type': 'AttributeMap', 'identifier': 'attributes'}]}, {'type': 'sequence', 'typeSequence': 'NodeInformation', 'name': 'NodeInformationSequence'}, {'type': 'enum', 'name': 'ErrorType', 'content': ' NonExistingNode, NonExistingAttribute, NodeAlreadyExists, AttributeAlreadyExists, InvalidPath, InvalidEngine, InvalidValues, OperationInvalidNode, InternalError, Collision'}, {'type': 'exception', 'name': 'InnerModelManagerError', 'content': '\n          ErrorType err;\n          string text;\n        '}, {'type': 'struct', 'name': 'Colored3DPoint', 'structIdentifiers': [{'type': 'float', 'identifier': 'x'}, {'type': 'float', 'identifier': 'y'}, {'type': 'float', 'identifier': 'z'}, {'type': 'byte', 'identifier': 'r'}, {'type': 'byte', 'identifier': 'g'}, {'type': 'byte', 'identifier': 'b'}]}, {'type': 'sequence', 'typeSequence': 'Colored3DPoint', 'name': 'PointCloudVector'}, {'type': 'struct', 'name': 'Plane3D', 'structIdentifiers': [{'type': 'float', 'identifier': 'px'}, {'type': 'float', 'identifier': 'py'}, {'type': 'float', 'identifier': 'pz'}, {'type': 'float', 'identifier': 'nx'}, {'type': 'float', 'identifier': 'ny'}, {'type': 'float', 'identifier': 'nz'}, {'type': 'float', 'identifier': 'width'}, {'type': 'float', 'identifier': 'height'}, {'type': 'float', 'identifier': 'thickness'}, {'type': 'string', 'identifier': 'texture'}]}, {'type': 'struct', 'name': 'coord3D', 'structIdentifiers': [{'type': 'float', 'identifier': 'x'}, {'type': 'float', 'identifier': 'y'}, {'type': 'float', 'identifier': 'z'}]}, {'type': 'struct', 'name': 'Pose3D', 'structIdentifiers': [{'type': 'float', 'identifier': 'x'}, {'type': 'float', 'identifier': 'y'}, {'type': 'float', 'identifier': 'z'}, {'type': 'float', 'identifier': 'rx'}, {'type': 'float', 'identifier': 'ry'}, {'type': 'float', 'identifier': 'rz'}]}, {'type': 'struct', 'name': 'jointType', 'structIdentifiers': [{'type': 'Pose3D', 'identifier': 'pose'}, {'type': 'float', 'identifier': 'lx'}, {'type': 'float', 'identifier': 'ly'}, {'type': 'float', 'identifier': 'lz'}, {'type': 'float', 'identifier': 'hx'}, {'type': 'float', 'identifier': 'hy'}, {'type': 'float', 'identifier': 'hz'}, {'type': 'float', 'identifier': 'mass'}, {'type': 'float', 'identifier': 'min'}, {'type': 'float', 'identifier': 'max'}, {'type': 'string', 'identifier': 'axis'}, {'type': 'int', 'identifier': 'port'}]}, {'type': 'struct', 'name': 'meshType', 'structIdentifiers': [{'type': 'Pose3D', 'identifier': 'pose'}, {'type': 'float', 'identifier': 'scaleX'}, {'type': 'float', 'identifier': 'scaleY'}, {'type': 'float', 'identifier': 'scaleZ'}, {'type': 'int', 'identifier': 'render'}, {'type': 'string', 'identifier': 'meshPath'}]}, {'type': 'sequence', 'typeSequence': 'float', 'name': 'FloatSeq'}, {'type': 'struct', 'name': 'Matrix', 'structIdentifiers': [{'type': 'int', 'identifier': 'cols'}, {'type': 'int', 'identifier': 'rows'}, {'type': 'FloatSeq', 'identifier': 'data'}]}], 'sequences': [{'name': 'RoboCompInnerModelManager/NodeInformationSequence', 'type': 'sequence', 'typeSequence': 'NodeInformation'}, {'name': 'RoboCompInnerModelManager/PointCloudVector', 'type': 'sequence', 'typeSequence': 'Colored3DPoint'}, {'name': 'RoboCompInnerModelManager/FloatSeq', 'type': 'sequence', 'typeSequence': 'float'}], 'simpleSequences': [{'name': 'RoboCompInnerModelManager', 'strName': 'NodeInformationSequence'}, {'name': 'RoboCompInnerModelManager', 'strName': 'PointCloudVector'}, {'name': 'RoboCompInnerModelManager', 'strName': 'FloatSeq'}], 'structs': [{'name': 'RoboCompInnerModelManager/AttributeType', 'type': 'struct', 'structIdentifiers': [['string', 'type'], ['string', 'value']]}, {'name': 'RoboCompInnerModelManager/NodeInformation', 'type': 'struct', 'structIdentifiers': [['string', 'id'], ['string', 'parentId'], ['NodeType', 'nType'], ['AttributeMap', 'attributes']]}, {'name': 'RoboCompInnerModelManager/Colored3DPoint', 'type': 'struct', 'structIdentifiers': [['float', 'x'], ['float', 'y'], ['float', 'z'], ['byte', 'r'], ['byte', 'g'], ['byte', 'b']]}, {'name': 'RoboCompInnerModelManager/Plane3D', 'type': 'struct', 'structIdentifiers': [['float', 'px'], ['float', 'py'], ['float', 'pz'], ['float', 'nx'], ['float', 'ny'], ['float', 'nz'], ['float', 'width'], ['float', 'height'], ['float', 'thickness'], ['string', 'texture']]}, {'name': 'RoboCompInnerModelManager/coord3D', 'type': 'struct', 'structIdentifiers': [['float', 'x'], ['float', 'y'], ['float', 'z']]}, {'name': 'RoboCompInnerModelManager/Pose3D', 'type': 'struct', 'structIdentifiers': [['float', 'x'], ['float', 'y'], ['float', 'z'], ['float', 'rx'], ['float', 'ry'], ['float', 'rz']]}, {'name': 'RoboCompInnerModelManager/jointType', 'type': 'struct', 'structIdentifiers': [['Pose3D', 'pose'], ['float', 'lx'], ['float', 'ly'], ['float', 'lz'], ['float', 'hx'], ['float', 'hy'], ['float', 'hz'], ['float', 'mass'], ['float', 'min'], ['float', 'max'], ['string', 'axis'], ['int', 'port']]}, {'name': 'RoboCompInnerModelManager/meshType', 'type': 'struct', 'structIdentifiers': [['Pose3D', 'pose'], ['float', 'scaleX'], ['float', 'scaleY'], ['float', 'scaleZ'], ['int', 'render'], ['string', 'meshPath']]}, {'name': 'RoboCompInnerModelManager/Matrix', 'type': 'struct', 'structIdentifiers': [['int', 'cols'], ['int', 'rows'], ['FloatSeq', 'data']]}], 'simpleStructs': [{'name': 'RoboCompInnerModelManager', 'strName': 'AttributeType'}, {'name': 'RoboCompInnerModelManager', 'strName': 'NodeInformation'}, {'name': 'RoboCompInnerModelManager', 'strName': 'Colored3DPoint'}, {'name': 'RoboCompInnerModelManager', 'strName': 'Plane3D'}, {'name': 'RoboCompInnerModelManager', 'strName': 'coord3D'}, {'name': 'RoboCompInnerModelManager', 'strName': 'Pose3D'}, {'name': 'RoboCompInnerModelManager', 'strName': 'jointType'}, {'name': 'RoboCompInnerModelManager', 'strName': 'meshType'}, {'name': 'RoboCompInnerModelManager', 'strName': 'Matrix'}], 'filename': '/opt/robocomp/interfaces/IDSLs/InnerModelManager.idsl'})
