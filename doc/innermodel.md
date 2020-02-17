# Introduction to InnerModel: RoboComp's internal representation of the world and the robot itself.

**InnerModel** is a C++ group of classes used by RoboComp to represent the world and the robot in it. As a data structure, InnerModel is a tree whose nodes represent entities in the world and geometric transformations among those entities. The current list of entities include:

*Geometry*
- innermodel
- transform
- translation
- rotation

*Bodies*
- mesh
- plane 

*Robots*
- differentialrobot
- omnirobot

*Actuators*
- prismaticjoint
- joint

*Sensors*
- laser
- pointcloud
- touchsensor
- display
- imu
- rgbd


A tree made up of elements (robots, bodies, sensors, and actuators) connected through 6D geometric transformations is a _kinematic tree_ . The basic relationship among elements is "part_of" meaning that a child is rigidly attached to her parent and will move with it. The precise position of the child with respect to her parent is encoded in an intermediate node that will typically be a _transform_ (6D geometric transformation). It also can be divided into a _translation_ or a _rotation_.

The InnerModel tree can be expressed as an XML file. This file can be written to disk at any time or read from a file when the components start. Once in memory, the InnerModel class holds a tree data structure and offers an API to access and modify its nodes and edges.

Let's start with a simple XML file describing a 5000mm x 5000mm square, a few boxes on it and a differential drive robot endowed with a laser and an RGBD camera: (the file can be found in ~/robocomp/files/innermodel/simplesimple.xml)

~~~~
<innerModel>
	<transform id="world">
		<transform id="floor" rx="3,14>
			<plane id="ddG" ny="1"  px="-0" py="0" pz="0" size="5000,5000,10" texture="/home/robocomp/robocomp/files/osgModels/textures/wood.jpg" /> 
		    
			<plane id="ddR" nx="1"  px="2500" py="200" pz="0" size="5000,500,10" texture="#eeeeee" />
			<plane id="ddL" nx="1" px="-2500" py="200" pz="0" size="5000,500,10" texture="#eeeeee" />	
			<plane id="ddF" nz="1"  pz="2500" py="200" px="0" size="5000,500,10" texture="#555555" />
			<plane id="ddB" nz="1" pz="-2500" py="200" px="0" size="5000,500,10" texture="#555555" />
		</transform>

		<!--OBSTACLES-->
		<transform id="caja1" tx="0" tz="1000" ty="0" >
			<plane id="cajaMesh1" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" />
		</transform>
			
		<differentialrobot id="base" port="10004">
			<mesh id="base_robex" file="/home/robocomp/robocomp/files/osgModels/robex/robex.ive"  tx="0" ty="0" tz="0" scale="1000" />
				<translation id="laserPose" tx="0" ty="140" tz="200">
					<laser id="laser" port="10003" measures="100" min="100" max="30000" angle="3" ifconfig="10000" />
						<plane id="sensorL" nz="1" pz="-200" size="100" repeat="1" texture="#ff0000" /> 
				</translation>	
				<transform id="camera" ty="210" tz="200">
						<rgbd id="rgbd" focal="490" width="640" height="480" port="10096" noise="0.00" ifconfig="40000,50000" />
						<plane id="rgbd_mesh1" nz="1" pz="-200" size="200,50,10" repeat="1" texture="#550000" />
				</transform>
				<translation id="robotGeometricCenter" tx="0" ty="0" tz="50">
				</translation>
			</differentialrobot>
	</transform>
</innerModel>
~~~~
The whole world is embraced inside the _innermodel_ tag. Inside it we find the following tags:
- 1 transform with id "world"
  - 1 transform with id "floor"
    - 1 plane for the floor object with nice wood texture
    - 4 planes for the walls
    - 1 transform for the object
      - 1 plane(box) for the object
    - 1 differentialrobot object
        - 1 mesh for the robot's body representation
    - 1 translation to position the laser
      - 1 laser
      - 1 plane for the laser's body representation
      and so on for the rgbd camera ...
      
Each of these elements is read into memory and corresponding classes are instantiated. Geometric transforms are encoded as RT matrices using a left-hand reference system with Y-axis pointing up, Z-axis pointing front and X-axis pointing right.

InnerModel's API provides methods to:
- access the nodes and their specific APIs
- add and remove new nodes
- compute a kinematic transformation between any two nodes in the tree. This transformation is read as " given a 3D point in some A node reference system, compute how it is seen from the reference system of node B". This is, by large, the most used method of the InnerModel API.

Perceptive components typically extend or update InnerModel after processing data coming from sensors.

A simple example of the use of InnerModel in a generated component is when we want to create a controller that drives the robot to reach some 2D point on the floor. The easiest way is to transform the world coordinates of the target into the robot's reference system. This transformed point is actually a vector coming out from the robot and reaching the target. We only need to compute the angle between this vector and the robot heading direction and use it to set a correcting rotation speed that will eventually align the robot with the target. At the same time, the module of that vector can be a good magnitude to control the advance speed of the robot. 
