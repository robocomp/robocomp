#Using robocompdsl: the command line component generator

**robocompdsl** is the new tool used in RoboComp to automatically generate components and modify their main properties once they have been generated (e.g., communication requirements, UI type). It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away.

This new version can only be used from the command line, but the languages used to define components and their interfaces remain mostly the same: **CDSL** to specify components and **IDSL** to specify interfaces. The only difference with the old RoboCompDSLEditor tool is that the reserved keywords (are now case independent). Take a look to the tutorial ["a brief introduction to Components"](components.md) for an introduction to the concept of component generation and the languages involved.

There are three tasks we can acomplish using **robocompdsl**: 

* generating a CDSL template file
* generating the code for a previously existing CDSL file
* regenerating the code for an already generated component.

## Generating a CDSL template file
Even though writing CDSL files is easy --their structure is simple and the number of reserved words is very limited-- robocompdsl can generate template CDSL files to be used as a guide when writing CDSL files.
Start by creating a new directory for your component named, for instance, *mycomponent*. Then run the code generator:

    $ robocompdsl path/to/mycomponent/mycomponent.cdsl

This will generate a CDSL file with the following content:

    import "/robocomp/interfaces/IDSLs/import1.idsl";
    import "/robocomp/interfaces/IDSLs/import2.idsl";
    Component CHANGETHECOMPONENTNAME
    {
    	Communications
    	{
    		implements interfaceName;
    		requires otherName;
    		subscribesTo topicToSubscribeTo;
    		publishes topicToPublish;
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };

The CDSL language is described in the tutorial ["A brief introduction to Components"](components.md). Just don't forget to change the name of the component.

 
## Generating a component given a CDSL file
 
Let's change the template file above by something like this, 

    import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
    import "/robocomp/interfaces/IDSLs/Laser.idsl";
    Component MyFirstComp{
        Communications{
            requires DifferentialRobot, Laser;
        };
    gui Qt(QWidget);
    language Cpp; //language Python;
    };
    
and save it as *mycomponent.cdsl*. Now run again robocompdsl with the CDSL file as first argument and the directory where the code should be placed as the second argument.

From the component's directory:
    
    cd path/to/mycomponent
    robocompdsl mycomponent.cdsl .

Watch the dot at the end!

These commands will generate the C++ (or Python) code in the specified directory.

## Modfiying the component to write a simple controller for the robot
Check that the *rcis* simulator is up and running. You should have two open windows, one with a camara looking at the world and another with the subjective camera of the robot. In not, in a new terminal type,

    cd ~/robocomp/files/innermodel
    rcis simpleworld.xml
    
Now, goto to the src subdirectory of the new component, 

    cd path/to/mycomponent/src
    
and open *specificworker.cpp* in your favorite editor. Go to the **void SpecificWorker::compute()** method and replace it with,
```
void SpecificWorker::compute( )
{
    const float threshold = 200; //millimeters
    float rot = 0.6;  //rads per second

    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
        
	if( ldata.front().dist < threshold)
	{
		std::cout << ldata.front().dist << std::endl;
 		differentialrobot_proxy->setSpeedBase(5, rot);
		usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
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
save and, 

##Added the cmake command 
    cd ..
    cmake .
    make
    
Note that we are using a lambda function as a parameter to the std::sort function so you will need a gcc compiler version equal or newer than 4.9. Check with gcc -v. If you don't have it, substitute the sort method with your own sorting procedure.

If you have generated the code using python the replace the *specificworker.py* found in the src folder with this code

		def compute(self):
			print 'SpecificWorker.compute...'
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
				print distance
				if distance < 400:
					self.differentialrobot_proxy.setSpeedBase(0, rot)
					time.sleep(1)
				else:
					self.differentialrobot_proxy.setSpeedBase(100, 0)
			except Ice.Exception, e:
				traceback.print_exc()
				print e
			return True

Save the file.

Now we need to tell the component where to find the DifferentialRobot and the Laser interfaces. Of course they are implemented by the rcis simulator, Run rcis <innemodel> and you can find the port numbers for each interface, so now we only need to change the ports in the configuration file. Copy the configuration file to your component home directory:

     cd path/to/mycomponent
     cp etc/config .
     gedit config
     
Change in the editor the port numbers located after *-p* 
     
    CommonBehavior.Endpoints=tcp -p 11000
    # Proxies for required interfaces
    LaserProxy = laser:tcp -h localhost -p 10003
    DifferentialRobotProxy = differentialrobot:tcp -h localhost -p 10004
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10

Save and 

    cd ..

Now start the component,

For c++,

    bin/mycomponent --Ice.Config=etc/config

For Python,

    python src/MyFirstComp.py --Ice.Config=etc/config

and watch the robot avoiding obstacles! 
Change the code to improve this simple behavior of the robot. Stop the component by closing its ui window, modify, recompile and execute again.

## Updating the source code of a component after modifying its CDSL file
Once we generated our component we might change our mind and decide to add a new connection to another interface or to publish a new topic. In these cases we can regenerate the code of the component just by changing the *.cdsl* file and executing again the command.

As you might have learned from the tutorial ["A brief introduction to Components"](components.md) RoboComp components are divided in specific code (files where you write your code) and generic code (autogenerated code which doesn't need to be edited). Running robocompdsl again on the same directory will ony overwrite these generic files. To ensure robocompdsl doesn't overwrite the changes you made to the specific files these are left unchanged, so the component might not compile after regeneration (e.g., you might need to add new methods).

##Video Tutorials. See in ["readthedocs"](http://robocomp.readthedocs.org/en/latest/robocompdsl/)

Component Creation using Python.


<iframe width="560" height="315" src="https://www.youtube.com/embed/9-fEKLgsgPc" frameborder="0" allowfullscreen></iframe>


Component Creation using C++


<iframe width="560" height="315" src="https://www.youtube.com/embed/LjHv_Nh16PU" frameborder="0" allowfullscreen></iframe>







