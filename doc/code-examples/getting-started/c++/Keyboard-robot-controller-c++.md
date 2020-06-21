# Control the directions of the bot using Keyboard

For this component. Import DifferentialRobot and build the component using robocompdsl. This has been explained in detail in the previous tutorials. Next write the algorithm in specificworker.cpp. The algorithm is as follows

1. Get input data from keyboard
2. Check the input and perform the actions accordingly.

We define a variable num and another character input which will store the entry from keyboard. istringstream is used to convert the string to the integer and store it in the variable num. Set the value of rot which is arbitrary here it is set to 0.3. In this example 1 is set as forward, 2 is set backward, 3 is left and 4 is right. The code is as follows,

```
void SpecificWorker::compute( )
{
	int num;
	char input[0];
	std::cout << " Enter \n 1: forward \n 3: left \n 2: backward \n 4: right" << endl;
	std::cin >> input;
	istringstream ( input ) >> num;
	float rot = 0.3

	if (num == 1 )
	{
		differentialrobot_proxy->setSpeedBase(300, 0);
  		usleep(750000);
		differentialrobot_proxy->setSpeedBase(0, 0);
  		usleep(500000);
	}
	if (num == 2)
	{
		differentialrobot_proxy->setSpeedBase(-300, 0);
  		usleep(750000);
		differentialrobot_proxy->setSpeedBase(0, 0);
  		usleep(500000);
	}
	if (num == 3)
	{
		differentialrobot_proxy->setSpeedBase(5, rot+0.8);
  		usleep(1000000);
		differentialrobot_proxy->setSpeedBase(0, 0);
  		usleep(500000);
	}
	if (num == 4)
	{
		differentialrobot_proxy->setSpeedBase(5, rot+0.8);
  		usleep(1000000);
		differentialrobot_proxy->setSpeedBase(0, 0);
  		usleep(500000);
	}
	else
	{
		std::cout << "invalid entry" << endl;
	}


}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
```

Save the code and build the component and simulate the innermodel simpleworld.xml and run the component by executing,

	bin/<componentname> --Ice.Config=etc/config

The entire component can be found [here](https://github.com/parasKumarSahu/robocomp-coding-examples/tree/master/Cpp-examples/keyboardRobotController)
