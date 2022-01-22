# Getting the distance measurement and displaying it in command window

### Component Generation
I will just run through the steps without much explanation as in detail explanation can be found in the previous tutorials.

	mkdir dist
	cd dist
	robocompdsl dist.cdsl

Editing your cdsl file, Importing DifferentialRobot and Laser.idsl

	import "Laser.idsl";
	import "DifferentialRobot.idsl";
	Component dist{
		Communications{
			requires DifferentialRobot, Laser;

		};
		language Cpp;
	};

Generating and Building the component

	robocompdsl dist.cdsl build
	cd build
	cmake .
	make

To program the component

	cd src
	gedit specificworker.cpp

Here we use Laser.idsl is used to get the distance from an obstacle. Hence we need to declare a vector that would store the distances that are measured. In this example it is called ldata

	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

The above line as discussed above gets the laser data, `getLaserData()` and stores it in `ldata`. Now since keeping into consideration that the bot will be moving and the distance has to be measured continuosly we sort the data that is been collected and stored in ldata by executing the code

```
std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
```

This code would sort and store the code in the first position or ldata.front(). So now to see the obstacle distance all we have to do is use std::cout in other words

	std::cout << ldata.front().dist << std::endl;


Now the entire code for this is

```
void SpecificWorker::compute( )
{


    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;


	differentialrobot_proxy->setSpeedBase(500, 0);
  	usleep(1500000);
	std::cout << ldata.front().dist << std::endl;
  	differentialrobot_proxy->setSpeedBase(10, 1.5707);
  	usleep(1000000);
	std::cout << ldata.front().dist << std::endl;

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

	bin/dist --Ice.Config=etc/config

You will now see the bot outputting the distance measurements on the command window.

Here the bot moves in random and you will see the results on the command window. The code for this component `Tutorial-3/thirdcomp` can be found on github [here](https://github.com/parasKumarSahu/robocomp-coding-examples/tree/master/Cpp-examples/dist)

As an exercise you can try building a obstacle avoiding bot using the same interfaces that you have learnt now.
