# Getting Distance and Angle Measurements

First we create a component, Refer previous tutorials where it is explained in detail.

For this example import both the DifferentialRobot and Laser interfaces hence your cdsl file should be

	import "/robocomp/interfaces/IDSLs/Laser.idsl";
	import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
	Component pydist{
		Communications{
			requires DifferentialRobot, Laser;

		};
		language Python;
	};

Generate the code using robocompdsl into a build folder. And navigate to src folder which is inside the build folder and open specificworker.py in any text editor.

Now you can edit the compute function in the file according to the problem statement. To get the distance and angle measurements we use laser interface and for movements we use DifferentialRobot interface

The algorithm is as follows we move the bot randomly first and then collect the laser data and store it in a defined vector say ldata. We then sort the distance data and take the first data of the vector list ldata. THe code is as below

	ldata = []
	d= []
	a =[]
	self.differentialrobot_proxy.setSpeedBase(0, 0.8)
	time.sleep(1)
	self.differentialrobot_proxy.setSpeedBase(50, 0)
	time.sleep(1)
	ldata = self.laser_proxy.getLaserData();
	for i in range(0,len(ldata)):
		dis = ldata[i]
		x = dis.angle
		y = dis.dist
		d.append(y)
		a.append(x)
	d.sort()
	a.sort()
	distance = d[0]
	angle = a[0]
	print(distance)
	print(angle)
	time.sleep(3)

As told in the algorithm we first define the ldata, Start moving the bot randomly and collect the laser data, sort the distance only by appending it to a new list. Take the first element for the new list and output the same on the command window. Your final compute function will look something like this

	def compute(self):
		print('SpecificWorker.compute...')
		try:
			ldata = []
			d= []
			a =[]
			self.differentialrobot_proxy.setSpeedBase(0, 0.8)
			time.sleep(1)
			self.differentialrobot_proxy.setSpeedBase(50, 0)
			time.sleep(1)
			ldata = self.laser_proxy.getLaserData();
			for i in range(0,len(ldata)):
				dis = ldata[i]
				x = dis.angle
				y = dis.dist
				d.append(y)
				a.append(x)
			d.sort()
			a.sort()
			distance = d[0]
			angle = a[0]
			print(distance)
			print(angle)
			time.sleep(3)
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)
		return True

Save it and run the component by executing

	python src/pydist.py --Ice.Config=etc/config

The distance and angle of the obstacles are outputted on the command window. The entire component can be found [here](https://github.com/rajathkumarmp/RoboComp-Python-Components/tree/master/LaserAngleCOmp)
