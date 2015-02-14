[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es)

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

#Installation in Ubuntu (14.04 and 14.10)

Make sure you have installed the following packages from the Ubuntu repository:

    apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev yakuake
    
Start yakuake by typing Alt-F2 and yakuake in the pop up window
Unroll yakuake terminal by pressing F12
    
###RoboComp core libraries

*cd* to your installation directory (if nothing better go to your home directory) and type:

    git clone https://github.com/robocomp/robocomp.git

Edit your ~/.bashrc file and add these lines at the end:

    export ROBOCOMP=<installation-directory>/robocomp
    export PATH=$PATH:/opt/robocomp/bin
   
reload bash by typing: 

    source ~/.basrhrc

Now we will create two symbolic links so RobComp can find everything. You will have to enter your passwd:

    sudo ln -s /opt/robocomp-1.0 /opt/robocomp
    sudo ln -s /home/<your-linux-user> /home/robocomp 

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

From *robocomp* root directory type:

    cd files/freedesktop
    chmod +x ./install.sh
    sudo ./install.sh
    cd ..
    git annex get .
    
It will take a little while to download all necessary textures...

Go now again to *robocomp* root directory and type:

    cd tools/rcinnermodelsimulator
    cmake .
    make
    sudo make install

Now let's run the simulator. Go to *robocomp* root directory and type:

    cd files/innermodel
    rcis betaworld.xml
    
Congratulations! RCIS should up and running with a simple robot endowed with a laser and an RGBD camera, moving on a wooden floor.
 
###Installing RoboLab's components from GitHub

Create another terminal in Yakuake and *cd* to *robocomp*. Then:

    cd components
    git clone https://github.com/robocomp/robocomp-robolab.git
    
RoboLab's set of basic robotics components are now dowloaded. 

###Connecting a JoyStick

If you have a JoyStick around, connect it to the USB port and:

    cd robocomp-robolab/components/joystickComp
    cmake .
    make
    cd bin
    ./startJoyStick.sh 
    
Your joystick should be now running and moving it will make the robot to advance and turn at your will.

###Creating a new component with RoboComp's DSLEditor
    
We will create now a new component that will connect to the RCIS simulator and run a simple controller for the robot, using the laser data. First we need to install the DSLEditor software that is runtime Eclipse application. 

Create another terminal in Yakuake and *cd* to *robocomp* and *cd* again to your *robocomp* root directory. Then type:

    cd tools
    python fetch_DSLEditor.py
    
Select 32 or 64 bits according to your current linux installation. After a little while the DSLEditor will be installed under the *robocompDSL* directory:

    cd robocompDSL/DSLEditor
    ./DSLEditor
    
Check that you have a *RoboComp* tab in the upper bar and that the *robocomp* directory appears in the Project Explorer. If it does not, right click in the *Project Explorer* pane and select *import*. Then select *General* and then *Existing Projects into Workspace*. Then select your *robocomp* directory and push *Finish*. 

Now we need to bring up some handy tabs in the lowe pane. Select *Window* in the upper bar, then "Show View" and *Other* and again *Other*. Select *Interfaces* and double-click on it.

We are ready now. Let's open a simple test component created for this example. Unfold the *robocomp* directory in the *Project Explorer* pane and then search for "Example.cdsl" and double click on it. The file will open in the syntax-sensitive editor. The first two lines import the two external interfaces that our component wil use:

    DifferentialRobot.idsl
    Laser.idsl
    
You can open them from the *robocomp/interfaces/IDSLs* directory to see how they are defined. 

The rest of the file sets the name of the component, *ExampleComp* and in the *Communications* sections defines its connectivity. *ExampleComp* **requires** two interfaces provided by two external components whose names will be specified later. *Requiring* means that our component will make remote procedure calls (RPC) to other components, passing them parameters and possibly obtaining in return some data.
    
More to come...
    
    
    
    



