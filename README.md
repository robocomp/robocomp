[RoboComp](http://robocomp.org)
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# About

An organization maintained by [RoboLab (Universidad de Extremadura)](http://robolab.unex.es), [Aston University](https://www2.aston.ac.uk/eas), [ISIS (Universidad de Málaga)](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=staff&Itemid=3&lang=es) and many other collaborators from the Google Summer of Code program.

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain-specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user-specific code is preserved thanks to a simple inheritance mechanism.

**If you already have RoboComp installed, jump to [tutorials](doc/README.md) to start coding!**

:warning: If you want to contribute with something new to Robocomp, please do it on the [development branch](https://github.com/robocomp/robocomp/tree/development).

:question: If you have a question please look for it in the [FAQ](doc/FAQ.md). 
-

- [Installation from source](#installation-from-source)
  * [Requirements](#requirements)
  * [Installation](#installation)
- [Testing the installation using the RCIS robotics simulator](#testing-the-installation-using-the-rcis-robotics-simulator)
  * [Installing some RoboLab's components from GitHub](#installing-some-robolab-s-components-from-github)
  * [Connecting a JoyStick (if no JoyStick available skip to the next section)](#connecting-a-joystick)
  * [Using the keyboard as a JoyStick](#using-the-keyboard-as-a-joystick)
- [Testing the installation using the Coppelia Simulator](#testing-the-installation-using-the--coppelia-simulator)
- [Next steps](#next-steps)
- [Known issues](#known-issues)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>




# Installation from source

Tested in Ubuntu 18.04, 19.10, 20.04 and 20.10.  
**Note:** RoboComp is not compatible with Ubuntu 16.04. RoboComp needs to be compiled using C++11. Ice libraries with C++11 support are only available for zeroc-ice 3.7 and the packages for this version are only available since Ubuntu 18.04.

**Note:** If you have installed Anaconda in your system. [Then you need to change the python from anaconda to default](https://github.com/robocomp/robocomp/issues/248).


## Requirements
Make sure you have installed the following packages from the Ubuntu repository:
```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends  \
      ca-certificates \
      cmake \
      curl \
      freeglut3-dev \
      g++ \
      gcc-multilib \
      git \
      git-annex \
      libboost-dev \
      libboost-system-dev \
      libboost-thread-dev \
      libgsl-dev \
      libopenscenegraph-dev \
      libqt5xmlpatterns5-dev \
      libqt5opengl5-dev \
      libzeroc-icestorm3.7 \
      make \
      python3 \
      python3-pip \
      python3-distutils \
      python3-pyparsing \
      python3-setuptools \
      python3-termcolor \
      python3-zeroc-ice \
      sudo \
      zeroc-ice-all-dev \
      zeroc-icebox 

sudo pip3 install pyside2 rich argcomplete prompt_toolkit

```

And to install the needed python packages:
```bash
sudo pip3 install pyside2 rich argcomplete prompt_toolkit

```

It is recommendable to install the following packages::

    sudo apt-get install yakuake qttools5-dev-tools qt5-assistant
    

## Installation

*cd* to your home directory (you are probably in it already) and type:

    git clone https://github.com/robocomp/robocomp.git

Now we will create a symbolic link so RoboComp can find everything. You will have to enter your password:

    sudo ln -sf ~ /home/robocomp
    
(the ~ is in Alt-4)
    
Edit your ~/.bashrc file 

    gedit ~/.bashrc

Add these lines at the end:

    export ROBOCOMP=~/robocomp
    export PATH=$PATH:/opt/robocomp/bin
   
make bash process the modified file by typing: 

    source ~/.bashrc

If you do not have Qt5 already installed, you may skip this step and proceed to compilation.

* Check Qt5 version used by qmake

```bash
qmake --version
```

If qmake uses a Qt5 version less than v5.15, you might want to manually install Qt5.15 or higher from [qt.io](https://www.qt.io/download-open-source) and configure qmake to use this version instead.

* To configure qmake, open the config file `/usr/share/qtchooser/default.conf` in your favourite text editor, add the following lines and save the configuration. (Run as root if needed)

```
<QT5-INSTALLATION-DIRECTORY>/bin
<QT5-INSTALLATION-DIRECTORY>/lib
```

* Update your `~/.bashrc`

```bash
gedit ~/.bashrc
```

* Add the following lines

```bash
export PATH=$PATH:<QT5-INSTALLATION-DIRECTORY>/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<QT5-INSTALLATION-DIRECTORY>/lib
```

* Update newly set environment variables

```bash
source ~/.bashrc
```

Done! Now let's compile and install the whole thing:

    sudo [ -d /opt/robocomp ] && rm -r /opt/robocomp
    cd robocomp
    mkdir build
    cd build
    cmake ..
    make
    sudo env "PATH=$PATH" make install

If you want to compile Robocomp with support for FCL, follow the instructions in the [Robocomp with FCL](doc/Compiling-RoboComp-with-collision-detection.md) tutorial.

The RoboComp's core libraries and simulator should now be compiled and installed in `/opt/robocomp`.

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

    cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickComp
    cmake .
    make
    cd bin
    sudo addgroup your-user dialout   // If you find permissions issues in Ubuntu
    check the config file in the component's etc folder and male sure that the port matches the DifferentialRobot endpoint in     RCIS.
    bin/joystick etc/config
    
Your joystick should be now running. It will make the robot advance and turn at your will. If it does not work, 
check where the joystick device file has been created (e.g., `/dev/input/js0`). If it is not `/dev/input/js0`, edit `~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickComp/etc/config` change it accordingly and restart. Note that you might want to save the *config* file to the component's home directory so it does not interfere with future GitHub updates.


## Using the keyboard as a JoyStick

If you don't have a JoyStick install this component,

    cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/keyboardrobotcontroller
    cmake .
    make
    src/keyboardrobotcontroller.py etc/config
    
and use the arrow keys to navigate the robot, the space bar to stop it and 'q' to exit.

Note 1: You must have your simulator running in a terminal and only then you can run a component in another terminal. You will get an error message if you run the above component without having RCIS already running.

Note 2: If you have anaconda installed (for python 3), It is recommended to uninstall anaconda first and then install robocomp. (It is only applicable if you have faced errors while running above commands.)

# Testing the installation using the Coppelia Simulator
We are now moving to more advanced robotics simulators that can reduce the gap between simulation and deployment. Our first choice now is [CoppeliaSim]((https://www.coppeliarobotics.com/) because it offers a scene editor that can be used during a running simulation, you can "hang" and modify Lua code from the scene nodes in no time, you can choose among 4 physics engines and, thanks to the [PyRep](https://github.com/stepjam/PyRep) library, we have a fast access to almost eveything running in the simulator.

To connect RoboComp and CoppeliaSim we use *bridges* that are Python components that include PyRep and implement/publish the required RoboComp interfaces. So far we have implemented three bridges that are located in this [repo](https://github.com/robocomp/dsr-graph/tree/development/robots_pyrep). These bridges interface scenes with Viriato, a simpler world using Viriato's omni base and an empty world with a Pioneer 2AT differential robot. The corresponding Coppelia .ttt files can be found [here](https://github.com/robocomp/dsr-graph/tree/development/etc).

To test RoboComp with Coppelia you need to:

- Install Coppelia Robotics and Pyrep. There are detailed instructions [here](https://github.com/robocomp/robocomp/blob/development/doc/DSR-start.md)
- Run the bridge, i.e. omniPyrep.py and see that Coppelia starts Ok.
- Connect your new component to the ports offered in omniPyrep.py or
- Connect a joystick or XBox pad to omniRep.py using [this component](https://github.com/robocomp/robocomp-robolab/tree/master/components/hardware/external_control/joystickpublish)
    


# Next steps

You can find more tutorials on RoboComp in [tutorials](doc/README.md) 

Drop comments and ask questions in:

- https://gitter.im/robocomp

Please, report any bugs with the github issue system: [Robocomp Issues](https://github.com/robocomp/robocomp/issues)

If you have any suggestions to improve the repository, like features or tutorials, please contact us on [![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) or create a feature request [here](https://github.com/robocomp/robocomp/issues).


# Known issues
- Compatibility problem between pyparsing version and Robocomp tools:
  * One of the main tools of Robocomp, robocompdsl is using pyparsing and the current code doesn't work with 2.4 version of this library. With the previous commands, we are installing the 2.2 version (python-pyparsing=2.2.0+dfsg1-2). If you have a more recent version of pyparsing installed with apt or pip we recommend you to uninstall it and install the 2.2 version. You can check your current version of pyparsing with this command:
```bash
python3 -c "import pyparsing; print(pyparsing.__version__)"
```
