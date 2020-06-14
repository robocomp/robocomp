rcbuildvalidator
===============================

This tools lets you validate the process of compilation and installation of any Robocomp Core branch (on github) in many different versions of Ubuntu releases. 

## Introduction
If you are developing you will have many times the need to test what you are doing on a clean system and maybe even on a different version of the one you are using on your machine.

Using [docker](https://www.docker.com/resources/what-container) we have created the *rcbuildvalidator* tool that allows any user to check if the installation of the robocomp core would work correctly on a newly installed and clean ubuntu system.

In addition, *rcbuildvalidator* allows you to open an interactive terminal where you can execute the installation instructions by yourself or run the robocomp_install.sh script from the home directory.

By default all images used with this tool do not save the changes after exiting. This helps to keep a clean system on which to test.

# Dependencies
rcbuildvalidator depends on Docker. On the first boot it checks if the Docker package and its dependencies are installed and if not, the user is offered the option to install it.

## Usage

```bash
$ rcbuildvalidator -h
usage: rcbuildvalidator [-h] [-b BRANCH] [-v VERSION] [--manual-mode] [-d]

rcbuildvalidator makes easy to test the installation and build of the Robocomp core
in many different Ubuntu versions.

optional arguments:
  -h, --help            show this help message and exit
  -b BRANCH, --branch BRANCH
  -v VERSION, --version VERSION
  --manual-mode
  -d, --debug
  -m [MOUNT [MOUNT ...]], --mount [MOUNT [MOUNT ...]]
```

If you run it without options, rcbuildvalidator tries the installation of Robocomp's "development" branch on an Ubuntu 18.04. The branch to be downloaded can be set with the *-b* or *--branch* option and the Ubuntu version can be set with *-v* or *--version*. This tool accepts auto-completion so you can type
```bash
rcbuildvalidator -v <TAB> <TAB>
```
and you will be offered with the options currently available (obtained from the Ubuntu tags available at https://hub.docker.com/_/ubuntu?tab=tags).

*--manual-mode* is perhaps one of the most interesting options . This allows the user to launch an interactive terminal of the Ubuntu version specified with -v. In addition, in the home page of the terminal that will appear, you can find the robocomp installation script robocomp_install.sh that you can use if you want to debug this process.

*-m* or *--mount* option lets you use one of your machine (host) directories or files to be mounted in the home directory of the "virtualized" system. This way it is possible to test any software you are currently developing inside a clean machine. It also helps to remember the dependencies of libraries and packages that your new piece of software have.

## TO-DO 
* It is planned to implement the option of being able to save with a proper name an image after having carried out the operations that are desired in the same one (installation of packages, configurations, etc)
* Check the available branches listed by github for Robocomp and been able to use any of those.