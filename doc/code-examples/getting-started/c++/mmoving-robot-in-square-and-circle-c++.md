# To simulate a bot to move in square

I would assume you have already learnt how to create a new component using robocompdsl if not you can learn about it [here](). Also, This tutorial requires an understanding of DifferentialRobot.idsl read about it [here]()

## Getting started
Create a new folder, say squarecomp

	mkdir squarecomp

First let us create a new component using command line tool, robocompdsl.

	cd squarecomp
	robocompdsl square.cdsl

This should have created a dummy cdsl file now edit the cdsl file

	gedit square.cdsl

The square.cdsl should import DifferntialRobot interface hence the cdsl file should look like this

	import "DifferentialRobot.idsl";
	Component square
	{
		Communications{
			requires DifferentialRobot;

		};
		language Cpp;
	};

Now save and generate the code by

	robocompdsl square.cdsl build

This should generate all the code in a new folder build. Now go to build and build the files

	cd build
	cmake .
	make

Not that you have successfully created the component let us start coding the component.

	cd src
	gedit specificworker.cpp

If you have already gone through the DIfferentialRobot documentation then you will be familiar with this line

	differentialrobot_proxy->setSpeedBase(x, y);

This sets the speed of the bot `x` in millimeters/sec and also to specify the rotating angle `y` in rad/sec for the bot to rotate.

NOw in specificworker.cpp, The `void SpecificWorker::compute( )` is where all the code needs to be written and where robocomp will know to 'compute' the code. Using delay function and `differentialrobot_proxy->setSpeedBase(x , y);` we can write

	differentialrobot_proxy->setSpeedBase(200, 0); // move 200 mm/sec frwd
  	usleep(1000000); //delay 1 second
  	differentialrobot_proxy->setSpeedBase(10, 1.5707);  //rotate 90 degrees at a speed of 10 mm/sec
  	usleep(1000000);  // delay 1 second

If you have understood the above code and written the same in teh specificworker.cpp then your code should look like this

	void SpecificWorker::compute( )
	{

    	try
    	{

  		differentialrobot_proxy->setSpeedBase(200, 0);
  		usleep(1000000);
  		differentialrobot_proxy->setSpeedBase(10, 1.5707);
  		usleep(1000000);

    	}
    	catch(const Ice::Exception &ex)
    	{
        	std::cout << ex << std::endl;   //If there is any error/exception display the same.
    	}

	}

Now we need to tell the component where to find the DifferentialRobot and the Laser interfaces.

```bash
cd etc/config .
gedit config
```

Change in the editor the port numbers located after *-p*

```bash
CommonBehavior.Endpoints=tcp -p 11000
# Proxies for required interfaces
LaserProxy = laser:tcp -h localhost -p 10003
DifferentialRobotProxy = differentialrobot:tcp -h localhost -p 10004
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
Ice.ACM.Client=10
Ice.ACM.Server=10
```

Save and

```bash
cd ..
```

Now save the file and build the component again

	cmake .
	make

To simulate how this code behaves we utilize the robocomp's already available innermodel. Open a new tab in the terminal and execute

	cd robocomp/files/innermodel
	rcis simpleworld.xml

This would start the simulator with the innermodel simpleworld. You will see two windows one the entire view of the map or model and the other is the camera view of the bot which is in the center. NOw go back to the firstcomp tab in the terminal and let us run the component.

	bin/square --Ice.Config=etc/config

The bot now moves in a square inside the innermodel simpleworld.

Now that you have understood and executed the bot to move in square. We can modify the same code and make the bot to move in circle.

For this, Change the parameters of the setSpeedBase to

	differentialrobot_proxy->setSpeedBase(10, 0.4);
  	usleep(1000000);

Save the file build it again and run it. And you will find it moving in circle. The code for the entire component can be found [here](https://github.com/parasKumarSahu/robocomp-coding-examples/tree/master/Cpp-examples/squarecomp)

This is the simulation of a basic components in robocomp. We will explore much complex components by learning more interfaces along this tutorial series.
