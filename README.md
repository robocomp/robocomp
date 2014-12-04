robocomp
========

RoboComp is a robotics framework providing a set of open-source, distributed, real-time robotic and artificial vision software components and the necessary tools to create and manage them.

It is mainly a response to the need of quickly creating and modifying software components. Currently, RoboComp is  request/reply native, but can also communicate through a publish/subscribe mechanism. It is not unusual that one component requires, subscribes, implements and publishes, all at once. To build components, RoboComp provides 2 domain specific languages: IDSL and PDSL, and a C++ generator. IDSL is an Interface Definition Language used to define interfaces, which represent abstract functionalities shown by components, in a similar way as an include file does for a class in C++. PDSL includes IDSLs to define the actual components, specifying the interfaces they implement, publish, require o subscribes to. With this information, the code generator creates a C++ source subtree, based on CMake, that compiles and executes flawlessly.

When some of these features have to be changed, the component can be easily regenerated and all the user specific code is preserved thanks to a clever inheritance mechanism.
