import unittest

from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool


class TestRoi(unittest.TestCase):

    # def test_communication_is_ice(self):
    #     """
    #     Test that check the creation of roi from frames
    #     """
    #
    #     communication_is_ice('')
    #     # def communication_is_ice(sb):
    #     # 	isIce = True
    #     # 	if len(sb) == 2:
    #     # 		if sb[1] == 'ros'.lower():
    #     # 			isIce = False
    #     # 		elif sb[1] != 'ice'.lower() :
    #     # 			print('Only ICE and ROS are supported')
    #     # 			sys.exit(-1)
    #     # 	return isIce

    def test_IDSLPool(self):
        pool = IDSLPool("AprilTags.idsl", [])
        self.assertIn("AprilTags", pool.modulePool)
        self.assertIn("GenericBase", pool.modulePool)
        self.assertIn("JointMotor", pool.modulePool)

        module = pool.moduleProviding("JointMotor")
        self.assertEqual(module['name'], 'RoboCompJointMotor')

        idsl_module = pool.IDSLsModule(module)
        self.assertEqual(idsl_module, '/opt/robocomp/interfaces/IDSLs/JointMotor.idsl')

        interfaces = pool.interfaces()
        self.assertEqual(interfaces, ['GenericBase', 'JointMotor', 'JointMotorPublish', 'AprilTags'])

        ros_imports = pool.rosImports()
        self.assertEqual(ros_imports, ['RoboCompGenericBaseROS/TBaseState', 'std_msgs/Int32', 'std_msgs/Float32', 'RoboCompJointMotorROS/MotorState', 'RoboCompJointMotorROS/MotorParams', 'RoboCompJointMotorROS/BusParams', 'RoboCompJointMotorROS/MotorGoalPosition', 'RoboCompJointMotorROS/MotorGoalVelocity', 'RoboCompJointMotorROS/MotorParamsList', 'RoboCompJointMotorROS/MotorGoalPositionList', 'RoboCompJointMotorROS/MotorGoalVelocityList', 'RoboCompJointMotorROS/MotorList', 'std_msgs/String', 'RoboCompAprilTagsROS/tag', 'RoboCompAprilTagsROS/tagsList'])

        ros_module_imports = pool.rosModulesImports()
        self.assertEqual(ros_module_imports, [{'strName': 'TBaseState', 'name': 'RoboCompGenericBase'}, {'strName': 'MotorState', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorParams', 'name': 'RoboCompJointMotor'}, {'strName': 'BusParams', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorGoalPosition', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorGoalVelocity', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorParamsList', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorGoalPositionList', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorGoalVelocityList', 'name': 'RoboCompJointMotor'}, {'strName': 'MotorList', 'name': 'RoboCompJointMotor'}, {'strName': 'tag', 'name': 'RoboCompAprilTags'}, {'strName': 'tagsList', 'name': 'RoboCompAprilTags'}])



    #
    # def test_roi_from_frame_bottom(self):
    #
    #     r = Roi.from_frame(frame, SIDE.BOTTOM, 100)
    #     self.assertEqual(r, [0, 0, 640, 480])
    #     r = Roi.from_frame(frame, SIDE.BOTTOM, 90)
    #     self.assertEqual(r, [0, 48, 640, 480-48])
    #     r = Roi.from_frame(frame, SIDE.BOTTOM, 10)
    #     self.assertEqual(r, [0, 480-48, 640, 48])
    #
    #
    # def test_roi_from_frame_left(self):
    #
    #     r = Roi.from_frame(frame, SIDE.LEFT, 100)
    #     self.assertEqual(r, [0, 0, 640, 480])
    #     r = Roi.from_frame(frame, SIDE.LEFT, 90)
    #     self.assertEqual(r, [0, 0, 640-64, 480])
    #     r = Roi.from_frame(frame, SIDE.LEFT, 10)
    #     self.assertEqual(r, [0, 0, 64, 480])
    #
    #
    # def test_roi_from_frame_right(self):
    #
    #     r = Roi.from_frame(frame, SIDE.RIGHT, 100)
    #     self.assertEqual(r, [0, 0, 640, 480])
    #
    #     r = Roi.from_frame(frame, SIDE.RIGHT, 90)
    #     self.assertEqual(r, [64, 0, 640 - 64, 480])
    #
    #     r = Roi.from_frame(frame, SIDE.RIGHT, 10)
    #     self.assertEqual(r, [640 - 64, 0, 64, 480])
    #
    #
    # def test_roi_from_frame_center(self):
    #
    #     r = Roi.from_frame(frame, SIDE.CENTER, 100)
    #     self.assertEqual(r, [0, 0, 640, 480])
    #     r = Roi.from_frame(frame, SIDE.CENTER, 90)
    #     self.assertEqual(r, [(64/2), (48/2), 640 - 64, 480-48])
    #     r = Roi.from_frame(frame, SIDE.CENTER, 10)
    #     self.assertEqual(r, [640/2 - 64/2, 480/2 - 48/2, 64, 48])
    #
    #
    # def test_roi_upscale(self):
    #     """
    #     Test that roi upscale right
    #     """
    #     roi = Roi([0,0,640,480])
    #     limit = Roi([0,0, 640,480])
    #     a = roi.upscaled(limit, 10)
    #     self.assertEqual(a, [0, 0, 640, 480])
    #
    #     roi = Roi([0, 0, 320, 240])
    #     a = roi.upscaled(limit, 10)
    #     self.assertEqual(a, [0, 0, 330, 250])
    #
    #     roi = Roi([10, 10, 320, 240])
    #     a = roi.upscaled(limit, 10)
    #     self.assertEqual(a, [5, 5, 330, 250])
    #
    #     roi = Roi([40, 40, 50, 50])
    #     limit = Roi([30, 30, 60, 60])
    #     a = roi.upscaled(limit, 10)
    #     self.assertEqual(a, [35, 35, 60, 60])


if __name__ == '__main__':
    unittest.main()