---
layout: post
title: Creating a new component with eclipse based RoboComp's DSLEditor
categories: [Tutorial]
tags: [DSLEditor]
description: We will create now a new component that will connect to the RCIS simulator and run a simple controller for the robot, using the laser data. First we need to install the DSLEditor software that is runtime Eclipse application...
---

#Creating a new component with Eclipse based RoboComp's DSLEditor
    
We will create now a new component that will connect to the RCIS simulator and run a simple controller for the robot, using the laser data. First we need to install the DSLEditor software that is runtime Eclipse application. 

Create another terminal in Yakuake and type:

    cd ~/robocomp/tools
    python fetch_DSLEditor.py
    
Select 32 or 64 bits according to your current linux installation. After a little while the DSLEditor will be installed under the *robocompDSL* directory:

    cd roboCompDSL/DSLEditor
    ./DSLEditor
    

Check that you have a *RoboComp* tab in the upper bar of the DSLEditor window and that the *robocomp* directory appears in the Project Explorer (left panel). If it does not, right click inside the *Project Explorer* panel and select *import*. Then select *General* and then *Existing Projects into Workspace*. Then select your *robocomp* directory and push *Finish*. 

Now we need to bring up some handy tabs in the lower pane. Select the *Window* tab in the upper bar, then *Show View*, then *Other* and again *Other*. Select now *Interfaces* and double-click on it. Go back to the main window.

Now, in the left panel, unfold the *robocomp* directory down to *robocomp/components/* and then click on it with the right button. Select *New Folder* and enter *mycomponents* in the folder name. Do it again to create a new folder inside *mycomponents* named *myfirstcomp*. Select *myfirstcomp* and then click on the *RoboComp* tab in the upper bar of the main window. Select *Create CDSL file* and fill the requested name with *MyFirstComp.cdsl*
    
The new file will open inside a syntax-sensitive editor in the central panel. Ctrl-space gives you syntactically correct options. You can see the skeleton of a new empty component. Look for the tab *Interfaces* in the lower bar and select *DifferentialRobot.idsl*. Click on the green cross at the right of the bar to include it and accept when prompted in a pop-up window. You will see something like:

    import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
    Component PFLocalizerComp{
        Communications{
            };
            language Cpp;
    };

Repeat the same steps to include *Laser.idsl* and then add a *requires* statement inside de *Communications* section. The file now should look like this:

    import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
    import "/robocomp/interfaces/IDSLs/Laser.idsl";
    Component MyFirstComp{
        Communications{
            requires DifferentialRobot, Laser;
        };
    language Cpp;
    };

Save the file and click in the upper bar on the *RoboComp* tab. Select *Generate Code*. After a little while the new source tree for your *MyFirstComp* component will be created. You can go back now to Yakuake and create a new tab to compile it. Then:

    cd ~/robocomp/components/mycomponents/myfirstcomp
    cmake .
    make
    bin/myfirstcomp --Ice.Config=etc/generic_config
    
and there it is! your component is running. 

What! Dissapointed? Yeah, I know it does nothing, but it runs and it is yours! Now let's do some real programming.

Stop the component with Ctrl Z and then type:

    killall -9 myfirstcomp
    
Now start your favorite IDE. KDevelop will do it just fine and you have it already installed. Open it in another tab, from Ubuntu menu or with Alt-F2. Then:

    Click the *Project* tab in the upper bar
    Select *Open/Import Project*
    Navigate to ~/robocomp/components/mycomponents/myfirstcomp
    Select *Makefile* and open the project
    
In the *Project* panel to the left of the screen, navigate to *src* and there select *specificworker.cpp* and open it. Open also *specificworker.h*

Now replace the empty *void compute()* method with this compact version of the classic AVOID-FORWARD-STOP architecture proposed by R. Brooks in the late 80's:

    void SpecificWorker::compute( )
    {
        static	float rot = 0.1f;			// rads/sec
        static float adv = 100.f;			// mm/sec
        static float turnSwitch = 1;
        const float advIncLow = 0.8;		// mm/sec
        const float advIncHigh = 2.f;		// mm/sec
        const float rotInc = 0.25;			// rads/sec
        const float rotMax = 0.4;			// rads/sec
        const float advMax = 200;			// milimetres/sec
        const float distThreshold = 500; 	// milimetres
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
        if( ldata.front().dist < distThreshold) 
        {
            adv = adv * advIncLow; 
            rot = rot + turnSwitch * rotInc;
            if( rot < -rotMax) rot = -rotMax;
            if( rot > rotMax) rot = rotMax;
            differentialrobot_proxy->setSpeedBase(adv, rot);
        }
        else
        {
            adv = adv * advIncHigh; 
            if( adv > advMax) adv = advMax;
            rot = 0.f;
            differentialrobot_proxy->setSpeedBase(adv, 0.f);		
            turnSwitch = -turnSwitch;
        }	
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    }
    
To compile the fancy version of *std::sort* you will have to first add this line at the end of the file *CMakeListsSpecific.txt* located in the same *src* directory:

    ADD_DEFINITIONS( -std=c++11 )
    
and then type:

    cmake .
    make

Hereafter, Press F8 in KDevelop to compile and link. Then, go to Yakuake and restart the component. 

Let us take InnerModel *simpleworld.xml* as an example. Open a new tab in Yakuake and execute

	cd robocomp/files/innermodel
	rcis simpleworld.xml

Now you should see 2 windows. Now in Yakuake go back to tab where you had compiled *myfirstcomp* and run

	bin/myfirstcomp --Ice.Config=etc/generic_config

You should see the robot maneouvring aroung the box. Now is when Robotics begin! Try to modify the code to let the robot go pass the blocking boxes.
