# To move a bot in square and circle

I will just post the important steps that is to be followed in detail steps of python component generation can be found in [here]().

	cd place/where/component/has/to/be/created
	mkdir pysquare
	cd pysquare
	robocompdsl pysquare.cdsl

You have now generated a dummy cdsl file

	gedit pysquare.cdsl

Change the dummy code that is there to the given below. Here you will be importing DIfferentialRobot interface.

	import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
	Component pysquare
	{
		Communications{
			requires DifferentialRobot;

		};
		language Python;
	};

save and close the file and execute

	robocompdsl pysquare.cdsl build

This command will generate the python code in a new folder in this case the build directory.

	cd build

Here you will find the folders etc and src. etc contains the config file which is required for Ice and src contains the src folder where you will be programming your bot

	cd src
	gedit specificworker.py

Here in the compute() function you will write the desired algorithm for simulation of the bot. To move it in a square we use DifferentialRobot's setSpeedBase function.

	self.differentialrobot_proxy.setSpeedBase(100, 0)

This will set the speed of the bot to 100 and 0 radians per second. So to move it in a square the entire algorithm will be

	self.differentialrobot_proxy.setSpeedBase(100, 0)
	time.delay(1)
	self.differentialrobot_proxy.setSpeedBase(10, 1.5707)
	time.delay(1)

In the above code 1.5705 is ntn but 90 degrees written in radians per second and a 1 second delay is introduced or each setSpeedBase will execute for a second. Now you compute function should look like this

	def compute(self):
		print('SpecificWorker.compute...')
		try:
			self.differentialrobot_proxy.setSpeedBase(100, 0)
			time.sleep(1)
			self.differentialrobot_proxy.setSpeedBase(10, 1.5707)
			time.sleep(1)
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)
		return True

After you have written the algorithm as desired you can save the code and come back to the build folder

To simulate the code we need a scenario or a innermodel for this we open a new tab in the terminal and execute

	cd robocomp/files/innermodel
	rcis simpleworld.xml

now come back to build folder or the previous tab and execute

	python src/pysquare.py --Ice.Config=etc/config

this should run the component and you will be able to see the bot moving in a square.

Now that you have understood and created a basic python component you can now extend this code and make it move in a circle. The algorithm is as follows

	self.differentialrobot_proxy.setSpeedBase(10, 0.87)
	time.sleep(0.5)

Svae the component and run it similarly as you did for square. The code for the entire component can be found [here](https://github.com/rajathkumarmp/RoboComp-Python-Components)
