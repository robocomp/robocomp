[RoboComp](http://robocomp.net)
===============================

by [RoboLab](http://robolab.unex.es)

RoboComp is a Robotics framework providing a set of open-source, distributed, real-time robotic and artificial vision software components and the necessary tools to create and manage them. It is mainly a response to the need of quickly creating and modifying software components in Robotics that communicate through publish interfaces. Components may require, subscribe, implement or publish interfaces. To build components, RoboComp provides 2 domain specific languages: IDSL and PDSL, and C++ and Python code generator. IDSL is a language used to define interfaces representing abstract functionalities shown by components, in a similar way as an include file does for a class in C++. PDSL includes IDSLs to define the actual components, specifying the interfaces they implement, publish, require o subscribes to, and other additional parameters. With this information, the code generator creates a C++/Python source subtree, based on CMake, that compiles and executes flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a simple inheritance mechanism.

Installation
============





