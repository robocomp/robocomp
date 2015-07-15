#Roobocomp Workspace Model

###The recommended layout for development is as follows:

![ Robocomp workspace model](/website/img/workspace_model.jpg "Robocomp workspace model")

##Elements of workspace


###Workspace
The workspace is the folder inside which you are going to be actively developing. Keeping things in a folder with connected development helps keep separation of development models.

###Source space
The source space is the folder is where it will be expected to look for packages when building. This folder is easily identified as it is where the toplevel.cmake is linked from the catkin project. Each component should be in a direct subdirectory. if the directory contains a file named *IGNORE_COMP* the component will be ignored while building the workspace.

###Build Space
The build space is the folder in which cmake is invoked and generates artifacts such as the CMakeCache. A typical invocation of cmake will look like this when following the recommended layout.

    cmake ../src
    

This need not be a direct sub directory of workspace. It can be any where.

###Development Space
The development space is where build system generates the binaries and config files which are executable before installation. This should be a direst subdirectory of workspace. Currently the `devel space is merged with the source space`.

###Install Space
If make install is called this is the directory into which cmake will target all installations. This directory contains a file names *.rc_install* which contain a semi colon separated paths of workspaces which are installed to this install space. Please note that the robocomp istall path */opt/robocomp* can is also an install space.


