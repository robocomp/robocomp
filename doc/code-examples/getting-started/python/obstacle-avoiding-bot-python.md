# Obstacle avoiding bot

Generate a component which subscribes to DifferentialRobot and Laser interfaces. The tutorial for the same is described in detail in the previous tutorials.

In the newly generated component open the specificworker.py in a text editor and write the algorithm as follows

1. Get the distance between the obstacle and the robot. This is explained in the previous tutorial
2. Check if the condition is below the treshold distance
3. If yes then rotate the bot
4. If no proceed forward

Here rotation is set to a variable which keep changing in each loop. This is done so as to ensure same path is not traced again and again.

The code for the above algorithm is

	def compute(self):
		print('SpecificWorker.compute...')
		rot = 0.7
		try:
			ldata = []
			d = []
			ldata = self.laser_proxy.getLaserData();
			for i in range(0,len(ldata)):
				dis = ldata[i]
				y = dis.dist
				d.append(y)
			d.sort()
			distance = d[0]
			print(distance)
			if distance < 400:
				self.differentialrobot_proxy.setSpeedBase(0, rot)
				time.sleep(1)
				rot = rot+ 0.5
				if rot > 3:
					rot = 1
				self.differentialrobot_proxy.setSpeedBase(70, 0)
				time.sleep(1)
			else:
				self.differentialrobot_proxy.setSpeedBase(100, 0)
				time.sleep(1)
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)
		return True

Note that here the values set are arbitrary and can be changed according to your requirements. Save the file and in a new tab simulate a innermodel

	cd robocomp/files/innermodel
	rcis simpleworld.xml

execute the component

	python src/componentname.py --Ice.Config=etc/config

Now you will find the obstacle avoiding bot is successfully implemented. The code for the entire component can be found [here](https://github.com/rajathkumarmp/RoboComp-Python-Components/tree/master/ObstacleAvoidBot)

