# Obstacle Avoiding Bot

In this tutorial you will learn how to build a bot which avoids the obstacle moving in random.

### Component Generation
I will just run through the steps without much explanation as in detail explanation can be found in the previous tutorials.

	mkdir obstacle
	cd obstacle
	robocompdsl obstacle.cdsl

Editing your cdsl file, Importing DifferentialRobot and Laser.idsl

	import "Laser.idsl";
	import "DifferentialRobot.idsl";
	Component obstacle{
		Communications{
			requires DifferentialRobot, Laser;

		};
		language Cpp;
	};

Generating and Building the component

	robocompdsl obstacle.cdsl build
	cd build
	cmake .
	make

To program the component

	cd src
	gedit specificworker.cpp

Here we will be using both the interfaces we have learnt so far. The entire algorithm is as follows

1. set a treshold value for distance at which the bot should avoid the obstacle and angle to rotate
2. get laser data and store it in ldata
3. sort the data in ldata
4. if the distance is less than the set treshold data then move right
5. display the distance and move forward
6. increase the angle by any value so that next time it moves towards the left.
7. If the rotation angle reaches a maximum then set it back to the original value
8. If the distance more than the treshold than proceed forward.

If you implement this algorithm your specificworker.cpp should be

```
void SpecificWorker::compute( )
{

    const float threshold = 200;
    float rot = 1.5707;



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;


	 if( ldata.front().dist < threshold)
	{
 	differentialrobot_proxy->setSpeedBase(5, rot);
	usleep(1250000);
	std::cout << ldata.front().dist << std::endl;
	differentialrobot_proxy->setSpeedBase(200, 0);
	usleep(500000);
	rot = rot + 0.12;
	if( rot > 3 * 1.5707 )
	{
	 rot = 1.5707;
	}
	}

	else
	{
	differentialrobot_proxy->setSpeedBase(200, 0);
  	usleep(500000);
	std::cout << ldata.front().dist << std::endl;
  	}


    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }

}
```

To compile the std::sort you will have to first add this line at the end of the file CMakeListsSpecific.txt located in the same src directory:

    ADD_DEFINITIONS( -std=c++11 )

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


Now save the file and build it

	cmake .
	make

and then open another tab in the terminal so that you can run a innermodel

	cd files/innermodel
	rcis simpleworld.xml

come back to the previous tab of the terminal and now run the above component by executing

	bin/obstacle --Ice.Config=etc/config

You will now see the bot moving around avoiding obstacle and outputting the distance measurements on the command window. The code for this component can be found [here](https://github.com/parasKumarSahu/robocomp-coding-examples/tree/master/Cpp-examples/obstacle)
