[RoboComp](http://robocomp.org)
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

by [RoboLab (Universidad de Extremadura)](http://robolab.unex.es), [Aston University](https://www2.aston.ac.uk/eas), [ISIS (Universidad de Málaga)](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=staff&Itemid=3&lang=es) and many other collaborators from the Google Summer of Code program.

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain-specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user-specific code is preserved thanks to a simple inheritance mechanism.

If you already have RoboComp installed, jump to [tutorials](doc/README.md) to start coding! 

:warning: If you want to contribute with something new to Robocomp, please do it on the [development branch](https://github.com/robocomp/robocomp/tree/development). If you fix something on the stable branch, please check if it's also relevant for the development branch and try to [cherrypick](https://git-scm.com/docs/git-cherry-pick) your commit.  

:question: If you have a question please look for it in the [FAQ](doc/FAQ.md). 
-


# Installation in Ubuntu from PPA

Coming soon...
<!--If you are not planning on modifying RoboComp itself (its libraries or tools), there's no need to go through all the compilation process. In this case, Ubuntu users of versions from 14.10 to 15.04 can install a packaged version of RoboComp. Just run the following commands:

    sudo add-apt-repository  ppa:imnmfotmal/robocomp
    sudo apt-get update
    sudo apt-get install robocomp

Remember to start a new bash session before continue using RoboComp: new variables included must be included in your shell environment.
-->

# Installation from source

Tested in Ubuntu 18.04.  
**Note:** RoboComp is not compatible with Ubuntu 16.04. RoboComp needs to be compiled using C++11. Ice libraries with C++11 support are only available for zeroc-ice 3.7 and the packages for this version are only available since Ubuntu 18.04.

**Note:** If you have installed Anaconda in your system. [Then you need to change the python from anaconda to default](https://github.com/robocomp/robocomp/issues/248).
<!--If you are not an Ubuntu user, need to modify the core of RoboComp, or just feel like installing from sources, you can follow these instructions (they have been tested in Ubuntu 14.04, 14.10, 15.04, 16.04). If you're not in any of these scenarios, please use the packaged version.
-->

## Requirements
Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get update
    sudo apt-get install git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice-all-dev freeglut3-dev libboost-system-dev libboost-thread-dev qt5-default libqt5xmlpatterns5-dev libqt5opengl5-dev libxt-dev libboost-test-dev libboost-filesystem-dev  libccd-dev zeroc-ice-all-runtime
    sudo apt-get install python3-pip python3-setuptools python3-pyparsing=2.2.0+dfsg1-2 python3-numpy python3-libxml2 python3-xmltodict python3-zeroc-ice
    sudo pip3 install networkx pyside2 argcomplete termcolor cogapp

It is recommendable to install the following packages::

    sudo apt-get install yakuake qttools5-dev-tools qt5-assistant

<!--Note: RoboComp uses python2 for now, so make sure that you set python2 as your default choice if you also have python3 installed on your system. You can do so by adding this line into your ~/.bashrc file and then save it:

    alias python=python2
--> 
Note: One of the main tools of Robocomp, robocompdsl is using pyparsing and the current code doesn't work with 2.4 version of this library. With the previous commands, we are installing the 2.2 version (python-pyparsing=2.2.0+dfsg1-2). If you have a more recent version of pyparsing installed with apt or pip we recommend you to uninstall it and install the 2.2 version. You can check your current version of pyparsing with this command:

    python3 -c "import pyparsing; print(pyparsing.__version__)"
    

## Installation itself

*cd* to your home directory (you are probably in it already) and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic link so RoboComp can find everything. You will have to enter your password:

    sudo ln -s ~ /home/robocomp
    
(the ~ is in Alt-4)
    
Edit your ~/.bashrc file 

    gedit ~/.bashrc

Add these lines at the end:

    export ROBOCOMP=~/robocomp
    export PATH=$PATH:/opt/robocomp/bin
   
make bash process the modified file by typing: 

    source ~/.bashrc

Done! Now let's compile and install the whole thing:

    sudo [ -d /opt/robocomp ] && rm -r /opt/robocomp
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
  
You can also use the default `innermodel/simpleworld.xml` anywhere if you have set the ROBOCOMP environment variable.

    rcis
    
Congratulations! RCIS should be up and running with a simple robot endowed with a laser and an RGBD camera, moving on a wooden floor. Don't forget to turn around the floor to see the robot from above.
 
#### Installing some RoboLab's components from GitHub

The software of the robots using RoboComp is composed of different software components working together, communicating among them. What we just installed is just the core of RoboComp (the simulator, a component generator, and some libraries). To have other features like joystick control we have to run additional software components available from other repositories, for example, robocomp-robolab:

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
    
and check where the joystick device file has been created (e.g., `/dev/input/js0`). If it is not `/dev/input/js0`, edit `~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickComp/etc/config` change it accordingly and restart. Note that you might want to save the *config* file to the component's home directory so it does not interfere with future GitHub updates.


## Using the keyboard as a JoyStick

If you don't have a JoyStick install this component,

    cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/keyboardrobotcontroller
    cmake .
    make
    src/keyboardrobotcontroller.py etc/config
    
and use the arrow keys to navigate the robot, the space bar to stop it and 'q' to exit.

Note 1: You must have your simulator running in a terminal and only then you can run a component in another terminal. You will get an error message if you run the above component without having RCIS already running.

Note 2: If you have anaconda installed (for python 3), It is recommended to uninstall anaconda first and then install robocomp. (It is only applicable if you have faced errors while running above commands.)

---------------------------------------------------------------------
You can find more tutorials on RoboComp in [tutorials](doc/README.md) 

Drop comments and ask questions in:

- https://groups.google.com/forum/?hl=en#!forum/robocomp-dev
- https://gitter.im/robocomp

Please, report any bugs with the github issue system: [Robocomp Issues](https://github.com/robocomp/robocomp/issues)

If you have any suggestions to improve the repository, like features or tutorials, please contact us on [![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) or create a feature request [here](https://github.com/robocomp/robocomp/issues).



    
    
    



