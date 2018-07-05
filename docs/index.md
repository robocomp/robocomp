[RoboComp](http://robocomp.org)
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

by [RoboLab](http://robolab.unex.es), [ISIS](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=staff&Itemid=3&lang=es) and many other collaborators.

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

If you already have RoboComp installed, jump to [tutorials](doc/README.md) to start coding! 


# Installation from source

Tested in Ubuntu 14.04, 14.10, 15.04 and 15.10
<!--If you are not an Ubuntu user, need to modify the core of RoboComp, or just feel like installing from sources, you can follow these instructions (they have been tested in Ubuntu 14.04, 14.10 and 15.04). If you're not in any of these scenarios, please use the packaged version.
-->

## Requirements
Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get update
    sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev pyqt4-dev-tools libboost-test-dev libboost-filesystem-dev python-libxml2 python-xmltodict libccd-dev
    sudo pip install networkx

It is recommendable to install the following packages::

    sudo apt-get install yakuake qt4-designer

## Installation itself

*cd* to your home directory (you are probably in it already) and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic link so RoboComp can find everything. You will have to enter your password:

    sudo ln -s /home/<your-linux-user> /home/robocomp 
    
Edit your ~/.bashrc file 

    gedit ~/.bashrc

Add these lines at the end:

    export ROBOCOMP=/home/<your-linux-user>/robocomp
    export PATH=$PATH:/opt/robocomp/bin
   
make bash process the modified file by typing: 

    source ~/.bashrc

Done! Now let's compile and install the whole thing:

    sudo rm -r /opt/robocomp
    cd robocomp
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

If you want to compile Robocomp with support for FCL, follow the instructions in the [Robocomp with FCL](doc/Compiling-RoboComp-with-collision-detection.md) tutorial.

The RoboComp's core libraries and simulator should now be compiled and installed in `/opt/robocomp`.

Let's now tell Linux where to find RoboComp's libraries:

    sudo nano /etc/ld.so.conf

and add the following line:

    /opt/robocomp/lib/
   
save the file and type:

    sudo ldconfig

Done! Now let's have some fun.

# Testing the installation using the RCIS robotics simulator
We will first fetch some meshes and textures used by the simulator (it will take a while):

    cd ~/robocomp
    git annex get .
    
Now let's run the simulator. 

    cd ~/robocomp/files/innermodel
    rcis simpleworld.xml
    
Congratulations! RCIS should be up and running with a simple robot endowed with a laser and an RGBD camera, moving on a wooden floor. Don't forget to turn around the floor to see the robot from above.
 
#### Installing some RoboLab's components from GitHub

The software of the robots using RoboComp is composed of different software components working together, communicating among them. What we just installed is just the core of RoboComp (the simulator, a component generator and some libraries). To have other features like joystick control we have to run additional software components available from other repositories, for example robocomp-robolab:

    cd ~/robocomp/components
    git clone https://github.com/robocomp/robocomp-robolab.git
    
The RoboLab's set of basic robotics components are now dowloaded. You can see them in `~/robocomp/components/robocomp-robolab/components`

## Connecting a JoyStick (if no JoyStick available skip to the next section)

If you have a joystick around, connect it to the USB port and:

    cd ~/robocomp/components/robocomp-robolab/components/joystickComp
    cmake .
    make
    cd bin
    sudo addgroup your-user dialout   // To solve some permissions issues in Ubuntu
    ./startJoyStick.sh 
    
Your joystick should be now running. It will make the robot advance and turn at your will. If the component does not start or the robot does not move stop joystickcomp with:

    ./forceStopJoyStickComp.sh
    
and check where the joystick device file has been created (e.g., `/dev/input/js0`). If it is not `/dev/input/js0`, edit `~/robocomp/components/robocomp-robolab/components/joystickComp/etc/config` change it accordingly and restart. Note that you might want to save the *config* file to the component's home directory so it does not interfere with future github updates.


## Using the keyboard as a JoyStick

If you don't have a JoyStick install this componentent,

    cd ~/robocomp/components/robocomp-robolab/components/keyboardrobotcontroller
    cmake .
    make
    src/keyboardrobotcontroller.py --Ice.Config=etc/config
    
and use the arrow keys to navigate the robot, the space bar to stop it an 'q' to exit.


---------------------------------------------------------------------
You can find more tutorials on RoboComp in [tutorials](doc/README.md)! 

Drop comments and ask questions in:

- https://groups.google.com/forum/?hl=es#!forum/robocomp-dev
- https://gitter.im/robocomp

Please, report any bugs to pbustos@unex.es

If you have any suggestions to improve the repository, like features or tutorials, please contact: robocomp.team@gmail.com 



    
    
    


