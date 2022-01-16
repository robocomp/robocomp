# Using robocompdsl: the command line component generator

**robocompdsl** is the new tool used in RoboComp to automatically generate components and modify their main properties once they have been generated (e.g., communication requirements, UI type). It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away.

This new version can only be used from the command line, but the languages used to define components and their interfaces remain mostly the same: **CDSL** to specify components and **IDSL** to specify interfaces. The only difference with the old RoboCompDSLEditor tool is that the reserved keywords (are now case independent). Take a look to the tutorial "[a brief introduction to Components](components.md)" for an introduction to the concept of component generation and the languages involved.

There are three tasks we can acomplish using **robocompdsl**. We can generate:

- A CDSL template file.
- The code for a previously existing CDSL file.
- The code for an already generated component.

&rightarrow; There are video tutorials at the end, if you prefer seeing someone else doing it first  :video_camera:

## Generating a CDSL template file
Even though writing CDSL files is easy -their structure is simple and the number of reserved words is very limited- robocompdsl can generate template CDSL files to be used as a guide when writing CDSL files.
Start by creating a new directory for your component named, for instance, *myComponent*, inside the robocomp directory. Then run the code generator:

```bash
$ mkdir myComponent	
$ robocompdsl myComponent/myComponent.cdsl
```

This will generate a CDSL file with the following content:

```bash
import "import1.idsl";
import "import2.idsl";

Component myComponent
{
        Communications
        {
                implements interfaceName;
                requires otherName;
                subscribesTo topicToSubscribeTo;
                publishes topicToPublish;
        };
        language Cpp//Cpp11//Python;
        gui Qt(QWidget//QDialog//QMainWindow);
	//options agmagent;
	//options InnerModelViewer;
};
```

The CDSL language is described in the tutorial "[A brief introduction to Components](components.md)". Just don't forget to change the name of the component so that it explains what it does.

 
## Generating a component given a CDSL file
 
Let's change the template file above by something like this, 

```cpp
    import "DifferentialRobot.idsl";
    import "Laser.idsl";
    Component MyFirstComp
    {
        Communications
	{
            requires DifferentialRobot, Laser;
        };
    language Cpp; // or language Python;
    };
```    
    
and save it as *myComponent.cdsl*. Now run again robocompdsl with the CDSL file as first argument and the directory where the code should be placed as the second argument.

From the component's directory:
    
```bash
cd path/to/myComponent
# There is a dot at the end of the following line!
robocompdsl myComponent.cdsl .
```

**Watch the dot at the end!**

These commands will generate the C++ (or Python) code in the specified directory.

NOTE 1:The current version for python 2.7 of the pyparsing module when installed from pip is 2.3 and it makes robocompcdsl to fail when generating the component.

NOTE 2: If you face errors in 'robocompdsl myComponent.cdsl .' command due to pyparsing version, Then follow 2 steps given below:
1. Current workaround is to uninstall the 2.3 version with pip :
```bash
sudo pip2 uninstall pyparsing
```

2. Install the 2.2 version from Ubuntu package (18.04) with :
```bash
sudo apt-get install pypy-pyparsing
```
NOTE 3: If installing pypy-parsing does not work and produces an error like :no module named pyparsing then run the following command : 
```bash
sudo pip2 install pyparsing==2.2
```
## Modifying the component to write a simple controller for the robot

Check that the *rcis simulator* is up and running. You should have two open windows, one with a camara looking at the world and another with the subjective camera of the robot. In not, in a new terminal type,

```bash
cd ~/robocomp/files/innermodel
rcis simpleworld.xml
```
    
Now, goto to the src subdirectory of the new component, 

```bash
cd path/to/mycomponent/src
```
    
and open *specificworker.cpp* in your favorite editor. Go to the **void SpecificWorker::compute()** method and replace it with,

```cpp
void SpecificWorker::compute( )
{
    const float threshold = 200; // millimeters
    float rot = 0.6;  // rads per second

    try
    {
    	// read laser data 
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 
	//sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });  
        
	if( ldata.front().dist < threshold)
	{
		std::cout << ldata.front().dist << std::endl;
 		differentialrobot_proxy->setSpeedBase(5, rot);
		usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(200, 0); 
  	}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}
```
save and, run cmake

## Added the cmake command 

```bash
    cd ..
    cmake .
    make
```

Note that we are using a lambda function as a parameter to the std::sort function so you will need a gcc compiler version equal or newer than 4.9. Check with `gcc -v`. If you don't have it, substitute the sort method with your own sorting procedure.

If you have generated the code using python then, open *specificworker.py* in your favorite editor. Go to the **def compute(self):** method and replace it with,
```python
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
			else:
				self.differentialrobot_proxy.setSpeedBase(100, 0)
	except Ice.Exception as e:
		traceback.print_exc()
		print(e)
	return True
```

Save the file and run cmake

```bash
    cd ..
    cmake .
    make
```

Now we need to tell the component where to find the DifferentialRobot and the Laser interfaces. Of course they are implemented by the rcis simulator, Run rcis <innemodel> and you can find the port numbers for each interface, so now we only need to change the ports in the configuration file. Copy the configuration file to your component home directory:


```bash
cd path/to/mycomponent
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
```

Save and

```bash
cd ..
```

Now start the component,

For C++:

```bash
bin/mycomponent --Ice.Config=etc/config
```

For Python:

```bash
python src/MyFirstComp.py --Ice.Config=etc/config
```
and watch the robot avoiding obstacles! 
Change the code to improve this simple behavior of the robot. Stop the component by closing its UI window, then modify, recompile and execute again.

## Updating the source code of a component after modifying its CDSL file
Once we generated our component we might change our mind and decide to add a new connection to another interface or to publish a new topic. In these cases we can regenerate the code of the component just by changing the *.cdsl* file and executing again the command.

As you might have learned from the tutorial "[A brief introduction to Components](components.md)" RoboComp components are divided in specific code (files where you write your code) and generic code (autogenerated code which doesn't need to be edited). Running robocompdsl again on the same directory will only overwrite these generic files. To ensure robocompdsl doesn't overwrite the changes you made to the specific files, these are left unchanged, so the component might not compile after regeneration (e.g., you might need to add new methods).

## Video Tutorials

### Component Creation using Python

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/9-fEKLgsgPc/0.jpg)](https://www.youtube.com/watch?v=9-fEKLgsgPc)

### Component Creation using C++

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/LjHv_Nh16PU/0.jpg)](https://www.youtube.com/watch?v=LjHv_Nh16PU)





