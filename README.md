[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es), [ISIS](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=staff&Itemid=3&lang=es) and many other collaborators.

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

#Installation in Ubuntu ( tested in 14.04 and 14.10)

Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get update
    sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools yakuake openjdk-7-jre python-pip  python-pyparsing python-numpy python-pyside pyside-tools
    
You might also want these nice developer tools:

    sudo apt-get install kdevelop vim aptitude nano 
    
Start Yakuake by typing Alt-F2, and then *yakuake* in the pop up window
Unroll the Yakuake terminal by pressing F12
    
###RoboComp core libraries

*cd* to your home directory (you are probably in it already) and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic link so RobComp can find everything. You will have to enter your passwd:

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
    sudo addgroup your-user dialout   //To solve some permissions issues un Ubuntu
    ./startJoyStick.sh 
    
Your joystick should be now running and moving it will make the robot to advance and turn at your will. If the component does not start or the robot does not move stop joystickcomp with:

    ./forceStopJoyStickComp.sh
    
and check where the JoyStick has been installed in /dev/input/jsX. If it is not /dev/input/js0, edit ~/robocomp/components/robocomp-robolab/components/joystickComp/etc/config change it accordingly and restart. Note that you might want to save the *config* file to the component's home directory so it does not interfere with future github updates.

Yo can find more tutorials on RoboComp in [tutorials!](doc/tutorials.md)

Please, report any bugs to pbustos@unex.es
    


    
    
    



