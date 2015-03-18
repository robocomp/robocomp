[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es), [ISIS](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=staff&Itemid=3&lang=es) and many other collaborators.

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

#Installation in Ubuntu (14.04 and 14.10)

Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get update
    sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools yakuake openjdk-7-jre python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev libgl1-mesa-dev-lts-utopic libglu1-mesa-dev pyqt4-dev-tools qt4-designer
    
You might also want these nice developer tools:

    sudo apt-get kdevelop vim aptitude nano 
    
Start Yakuake by typing Alt-F2, and then *yakuake* in the pop up window
Unroll the Yakuake terminal by pressing F12
    
###RoboComp core libraries

*cd* to your home directory (you are probably in it already) and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic link so RobComp can find everything. You will have to enter your passwd:

    sudo ln -s /home/<your-linux-user> /home/robocomp 
    
Edit your ~/.bashrc file 

    gedit ~/.bashrc

Add these lines at the end:

    export ROBOCOMP=<installation-directory>/robocomp
    export PATH=$PATH:/opt/robocomp/bin
   
reload bash by typing: 

    source ~/.bashrc

Done! Now let's compile the whole thing:

    cd robocomp
    cmake .
    make
    sudo make install
    
RoboComp's core libraries should now be compiled and installed in */opt/robocomp*

Let's now tell Linux where to find RoboComp's libraries:

    cd /etc
    sudo nano ld.so.conf

and add the following line:

    /opt/robocomp/libs/
   
save the file and type:

    sudo ldconfig

Done! Now let's have some fun.

###Installation of the RCIS Robotics simulator

    cd ~/robocomp
    cd files/freedesktop
    chmod +x ./install.sh
    sudo ./install.sh
    cd ..
    git annex get .
    
It will take a little while to download all necessary graphic textures used in the simulator...

And now compile the whole thing

    cd ~/robocomp/tools/rcinnermodelsimulator
    cmake .
    make
    sudo make install

Now let's run the simulator. 

    cd ~/robocomp/files/innermodel
    rcis simpleworld.xml
    
Congratulations! RCIS should up and running with a simple robot endowed with a laser and an RGBD camera, moving on a wooden floor. Don't forget to turn around the floor to see the robot from above.
 
###Installing RoboLab's components from GitHub

    cd ~/robocomp/components
    git clone https://github.com/robocomp/robocomp-robolab.git
    
The RoboLab's set of basic robotics components are now dowloaded. You can see them in ~/robocomp/components/robocomp-robolab/components

###Connecting a JoyStick

If you have a JoyStick around, connect it to the USB port and:

    cd ~/robocomp/components/robocomp-robolab/components/joystickComp
    cmake .
    make
    cd bin
    sudo addgroup tu-usuario dialout   //To solve some permissions issues un Ubuntu
    ./startJoyStick.sh 
    
Your joystick should be now running and moving it will make the robot to advance and turn at your will. If the component does not start or the robot does not move:

    stop joystickcomp with forceStopJoyStickComp.sh
    
and check where the JoyStick has been installed in /dev/input/jsX. If it is not /dev/input/js0, edit ~/robocomp/components/robocomp-robolab/components/joystickComp/etc/config change it accordingly and restart.

###Creating a new component with RoboComp's DSLEditor
    
We will create now a new component that will connect to the RCIS simulator and run a simple controller for the robot, using the laser data. First we need to install the DSLEditor software that is runtime Eclipse application. 

Create another terminal in Yakuake and type:

    cd ~/robocomp/tools
    python fetch_DSLEditor.py
    
Select 32 or 64 bits according to your current linux installation. After a little while the DSLEditor will be installed under the *robocompDSL* directory:

    cd robocompDSL/DSLEditor
    ./DSLEditor
    

Check that you have a *RoboComp* tab in the upper bar of the DSLEditor window and that the *robocomp* directory appears in the Project Explorer (left panel). If it does not, right click inside the *Project Explorer* panel and select *import*. Then select *General* and then *Existing Projects into Workspace*. Then select your *robocomp* directory and push *Finish*. 

Now we need to bring up some handy tabs in the lower pane. Select the *Window* tab in the upper bar, then *Show View*, then *Other* and again *Other*. Select now *Interfaces* and double-click on it. Go back to the main window.

Now, in the left panel, unfold the *robocomp* directory down to *robocomp/components/* and then click on it with the right button. Select *New Folder* and enter *mycomponents* in the folder name. Do it again to create a new folder inside *mycomponents* named *mycomp*. Select *myfirstcomp* and then click on the *RoboComp* tab in the upper bar of the main window. Select *Create CDSL file* and fill the requested name with *MyFirstComp.cdsl*
    
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

Hereafter, Press F8 in KDevelop to compile and link. Then, go to Yakuake and restart the component. You should see the robot maneouvring aroung the box. Now is when Robotics begin! Try to modify the code to let the robot go pass the blocking boxes.

###Maintaining your own repository of components

We recommend that you create a repository for your components (i.e. *mycomponents* directory in the example before) in your GitHub account (or other similar site) and pull/clone it in *~/robocomp/components* whenever yo need it. For example, if your GitHub account is *myaccount*, first log in with your browser and create a new repository named *mycomponents" following this instructions: 

    https://help.github.com/articles/create-a-repo/
    
Now is good time to write down a short description of what your component does in the README.md file.

Then we need to clean up the binary and generated files in *myfirstcomp*. Note that this is not necessary if you upload the component to the repo just after creating it with DSLEditor and before you type *cmake .*

    cd ~/robocomp/components/mycomponents/myfirstcomp
    make clean
    sudo rm -r CMakeFiles
    rm CMakeCache.txt
    rm cmake_install.cmake
    rm Makefile
    rm *.kd*
    rm src/moc*
    sudo rm -r src/CMakeFiles
    rm src/cmake_install.cmake
    rm src/Makefile
    
now we are ready:

    cd ~/robocomp/components/mycomponents
    git init
    git remote add origin "https://github.com/myaccount/mycomponents.git"
    git add mycomponents
    git push -u origin master
    
You can go now to GitHub and chek that your sources are there!
    
  
    

Yo can find more tutorials on RoboComp in http://robocomp.net

Please, report any bugs to pbustos@unex.es
    


    
    
    



