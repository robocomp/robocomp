# Command Line Interface 

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

These are command line tools. Each of them is both a Python library and an executable.
A more detailed description of each tool can be found in each of the directories, but here you can read a summary:

### Installation
This tools are installed as python packages.
```bash
cd ~/robocomp/tools/cli/
pip install .
```
 

### Commands
### `robocomp` (can be aliased to `rc` for simplicity).
This is the main tool that encompasses all the others. You can check the functionality of each option with the help of the --help option:

### `robocompdsl` (or `robocomp dsl`)
It is the tool used for the generation of new components and agents from .CDSL<link> files.

### `rcworkspace` (or `robocomp workspace`)
To facilitate the use of other tools, robocomp defines a workspace as a directory containing a set of components. This concept is relative and can be from an independent repository to a set of subdirectories of a project. The workspace concept is used to access and locate robocomp components on a machine.

### `rccd` (or `robocomp cd`)
When a project grows, the paths to access components can be quite complex and time consuming to move between them. rccd makes it easy to access these paths.

### `rcbuild` (or `robocomp build`) 
Tool to ease the process of building components and even to recompile and install robocomp itself.

### `rcrun` (or `robocomp run`)
Tool that allows the execution of components without the need to enter the directory and execute the binary directly. It can be used simply from the component name.

### `rcconfig` (or `robocomp config`)
Basic tool to configure some global variables to the rest of the tools. It is usually only run automatically during installation to register the directory where the robocomp source code has been downloaded.

### `rcdocker` (or `robocomp docker`)
Tool that acts as a docker wrapper, using its api and adapting some of the commands to create its own images.

### `rcportchecker` (or `robocomp portchecker`)
Most of the robocomp components use ZeroC-ICE based communications. These communications require ports that are configured in the corresponding files of each component. With this tool you can check the ports of the components as well as some communication interfaces.

### `install`(dir)
Contains scripts to facilitate the installation of Robocomp.