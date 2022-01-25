RoboComp: open-source Robotics framework
=========================================

RoboComp is an open-source Robotics framework providing the tools to create and modify software components that communicate through public interfaces. Components may *require*, *subscribe*, *implement* or *publish*
interfaces in a seamless way. Building new components is done using two domain-specific languages, IDSL and CDSL. With IDSL you define an interface and with CDSL you specify how the component will communicate with the world. With this information, a code generator creates C++ and/or Python sources, based on CMake, that compile and execute flawlessly. When some of these features have to be changed, the component can be easily regenerated and all the user-specific code is preserved thanks to a simple inheritance mechanism.

```{note}
This process has been **tested in Ubuntu 20.04 and 20.10**. While RoboComp may run in other distributions, it's highly discouraged to do so, as that hasn't been tested.

This document assumes you're running a compatible Ubuntu system.
```

Setup
------

RoboComp must be compiled from the source code. However, an installation script is provided to automate this process as much as possible. The first step is installing the script dependencies:

```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends  \
      sudo \
      curl \
      ca-certificates
```

```{warning}
Make sure you have deleted any previous versions of RoboComp from `/usr/local/bin/robocomp*` before performing an installation. Otherwise, conflicting files may (and will) cause problems in the long run!
```

To install the rest of robocomp you can use the script:

```bash
cd ~
curl -sL https://raw.githubusercontent.com/robocomp/robocomp/development/tools/install/robocomp_install.sh | bash -s
```

```{warning}
At some point, Eigen changed its include paths. As RoboComp has not been adapted to this change yet, a symlink is required to keep it working:

`sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen`
```

%{TODO: talk about FCL}

### Workspace setup

If you have followed the instructions above, you should have the files required to start using RoboComp in your system. The following tools are not required, but they make working with RoboComp much easier:

[yakuake](https://apps.kde.org/es/yakuake/)
:	Drop-down terminal emulator based on KDE Konsole technology. It makes handling tens of terminals much easier, supporting tabs and tiling layouts. Also, it can dump all that information (the _session_) to an script which can be called later to restore it.

[meld](https://meldmerge.org/)
:	Visual diff and merge tool targeted at developers. Meld helps you compare files, directories, and version controlled projects. It provides two- and three-way comparison of both files and directories, and has support for many popular version control systems.

To install them, just run:

```bash
sudo apt-get install yakuake qttools5-dev-tools qt5-assistant meld
```

Useful resources
-----------------

Tutorials
----------

- [A Brief introduction to Components](components.md)
- [Understanding RoboComp's Workspaces](workspaceModel.md)
- [Creating my first component using **robocompdsl**](robocompdsl.md)
- [IDSL: RoboComp's domain specific language for *interfaces* definition](IDSL.md)
- [CDSL: RoboComp's domain specific language for *components* definition](CDSL.md)
- [RoboComp's  build tools ](buildTools.md)
- [Starting with DSR](DSR-start.md)
- [List of RoboComp Interfaces](interfaces/README.md)
- [Introduction to InnerModel: RoboComp's internal representation of the world and itself](innermodel.md)
- [Maintaining your own repository of components](using_github.md)
- [How to contribute to RoboComp using the GitHub branching mechanism](contribute/contribute.md)
- [Coding examples](https://github.com/robocomp/robocomp-examples/blob/master/README.md)
- [Packaging Robocomp](packaging/packaging.md)
- [Compiling RoboComp with collision detection](Compiling-RoboComp-with-collision-detection.md)
- [RoboComp Component & Ros Node chatter](RobocompRosChatter.md)
- [RoboComp Integration with CoppeliaSim Using PyRep](robocomp-pyrep.md)

FAQ
----

Components
-----------

CORTEX
-------

DSR graph
----------

Agents
-------