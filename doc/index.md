RoboComp: open-source Robotics framework
=========================================

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain-specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user-specific code is preserved thanks to a simple inheritance mechanism.

```{note}
This process has been **tested in Ubuntu 20.04 and 20.10**. While RoboComp may run in other distributions, it's highly discouraged to do so, as they haven't been tested.

This document assumes you're running a compatible Ubuntu system.
```

%{Sections are included to keep this document from growing too large}

Setup
------

```{include} Setup.md
```

Useful resources
-----------------

```{include} Resources.md
```

Tutorials
----------

```{include} Tutorials.md
```

Frequently Asked Questions
---------------------------

```{include} FAQ.md
```

Components
-----------

%{TODO: as there is already a components.md out there, and the intro is pretty short, this section is inlined}

Robotics is a mixed bag of technology, where almost everything finds its way through. Also, Robotics is the place where our dreams of intelligent machines meet, in an endless attempt to build a truly useful tool for our daily lives. Because of this, we organize the software for our robots in big architectures that try to reproduce whatever we understand as intelligent behavior.

Components provide a new, developing technology that can be very helpful. Components are *programs that communicate* and as such, they are built with everything at hand: libraries, objects, threads, sockets, lambda functions and any other thing you can come up with to code a program. 

[RoboComp's components model](components.md) is quite simple and we always try to simplify it even more. It can be best explained through two Domain Specific Languages (DSLs) that have been created to define a component at a very high level of abstraction.

---

%{TODO: organize existing information and include relevant .md files}

CORTEX
-------

DSR graph
----------

Agents
-------