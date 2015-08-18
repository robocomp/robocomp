#Roobocomp Workspace Model

The workspace is for one who is developing the components rather than the framework itself.So why do even Robocomp needs a workspace when there is no trouble developing on it currently. The main reason for having a workspace is that it will make the work-flow much easier. Workspace basically organizes you development.  For example currently for building or running a component you have to go to its directory, create a build directory and then use cmake the build tools will automate all of this. Also as the number of components increases it would be very hard to keep track of all the components.

###The recommended layout for development is as follows:

![ Robocomp workspace model](../images/workspace_model.jpg "Robocomp workspace model")

##Elements of workspace


###Workspace
The workspace is the folder inside which you are going to be actively developing components. Keeping things in a folder with connected development helps keep separation of development models.On simple words, a workspace can be thought of as separating different components, for example Robocomp has some default components you may as well create some components so in this case you Robocomp's components can be in a workspace while your components in another. The config file in ~/.config/RoboComp/rc_worksapce.config maintains a list of all the registered workspaces.

###Source space
The source space (a folder inside workspace) contains the source code of all the components in the workspace or this is where you will be developing. The source space is the folder is where build tools will look for components. This folder is easily identified as it is where the toplevel.cmake is linked Robocomp installed folder. Each component should be in a direct subdirectory. If the directory contains a file named *IGNORE_COMP* the component will be ignored while building the workspace.

###Build Space
The build space is the folder in which cmake is invoked and generates artifacts such as the CMakeCache. A typical invocation of cmake will look like this when following the recommended layout.

    cd <build space folder>
    cmake ../src
    
This need not be a direct sub directory of workspace. It can be any where.

###Development Space
The development space is where build system generates the binaries and config files which are executable before installation. This should be a direst subdirectory of workspace. Currently the `devel space is merged with the source space` as you can seen in the layout graph.

###Install Space
If make install is called this is the directory into which cmake will target all installations. This directory contains a file names *.rc_install* which contain a semi colon separated paths of workspaces which are installed to this install space. Please note that the robocomp install path */opt/robocomp* is also an install space.

