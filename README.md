[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es)

RoboComp is a Robotics framework providing a set of open-source, distributed, real-time robotic and artificial vision software components and the necessary tools to create and manage them. It is mainly a response to the need of quickly creating and modifying software components in Robotics that communicate through publish interfaces. Components may require, subscribe, implement or publish interfaces. To build components, RoboComp provides 2 domain specific languages: IDSL and PDSL, and C++ and Python code generator. IDSL is a language used to define interfaces representing abstract functionalities shown by components, in a similar way as an include file does for a class in C++. PDSL includes IDSLs to define the actual components, specifying the interfaces they implement, publish, require o subscribes to, and other additional parameters. With this information, the code generator creates a C++/Python source subtree, based on CMake, that compiles and executes flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

#Installation in Ubuntu 14.04 and 14.10

Make sure you have installed the following packages from the Ubuntu repository:

    apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev

*cd* to your installation directory (if nothing better go to your home directory) and type:

    git clone https://github.com/robocomp/robocomp.git

Edit your ~/.bashrc file and add these lines at the end:

    export ROBOCOMP=<installation-directory>/robocomp
   
reload bash by typing: 

    source ~/.basrhrc

Now type *cd* robocomp

    cmake .
    make
    sudo make install
    
RoboComp's core libraries should be compiled and installed in */opt/robocomp*




