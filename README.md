[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es)

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

#Installation in Ubuntu (14.04 and 14.10)

Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get update
    sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools yakuake openjdk-7-jre kdevelop vim aptitude nano
    
Start Yakuake by typing Alt-F2 and yakuake in the pop up window
Unroll Yakuake terminal by pressing F12
    
###RoboComp core libraries

*cd* to your home directory and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic links so RobComp can find everything. You will have to enter your passwd:

    sudo ln -s /home/<your-linux-user> /home/robocomp 
    
Edit your ~/.bashrc file and add these lines at the end:

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

###Installation of RCIS Robotics simulator

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
    rcis betaworld.xml
    
Congratulations! RCIS should up and running with a simple robot endowed with a laser and an RGBD camera, moving on a wooden floor.
 
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
    
Check that you have a *RoboComp* tab in the upper bar of the window and that the *robocomp* directory appears in the Project Explorer (left panel). If it does not, right click in the *Project Explorer* panel and select *import*. Then select *General* and then *Existing Projects into Workspace*. Then select your *robocomp* directory and push *Finish*. 

Now we need to bring up some handy tabs in the lower pane. Select the *Window* tab in the upper bar, then *Show View*, then *Other* and again *Other*. Select now *Interfaces* and double-click on it. Go back to the main window.

Now, in the left panel, unfold the *robocomp* directory down to *robocomp/components/* and then click on it with the right button. Select *New Folder* and enter *mycomponents* in the folder name. Do it again to create a new folder inside *mycomponents* named *mycomp*. Select *myfirstcomp* and then click on the *RoboComp* tab in the upper bar of the main window. Select *Create CDSL file* and fill the requested name with *MyFirstComp.cdsl*
    
The new file will open inside a syntax-sensitive editor in the central panel. Ctrl-space gives you language available options. You can see the skeleton of a new empty component. Look for the tab *Interfaces* in the lower bar and select *DifferentialRobot.idsl*. Click on the green cross at the right of the bar to include it and accept when prompted in a pop-up window. You will see something like:

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
    cd bin
    ./startMyFirsComp.sh   (you might have to type: chmod +x *.sh , in the bin directory
    
and there it is! your component is running. 

What! Dissapointed? Yeah, I know it does nothing, but it runs and it is yours! Now let's do some real programming.

Stop the component with ./forceStopJoyStick.sh and start your favorite IDE. KDevelop will do it just fine and you have it already installed. Open it in another tab, from Ubuntu menu or with Alt-F2. Then:

    Click the *Project* tab in the upper bar
    Select *Open/Import Project*
    Navigate to ~/robocomp/components/mycomponents/myfirstcomp
    Select *Makefile* and open the project
    
In the *Project* panel to the left of the screen, navigate to *src* and there select *specificworker.cpp* and open it. Open also *specificworker.h*

Now copy this piece of code inside the *void compute()* method:

    code here
    
Press F8 to compile and link. Go to Yakuake and restart the component. You should see the robot maneouvring aroung the box. Now is when Robotics begin!


Yo can find more tutorials in http://robocomp.net
    


    
    
    



