rcdeploymentchecker
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What the `rcdeploymentchecker` does?

The `rcdeploymentchecker` is a tool for checking the deployment files used by rcmanager. This tool parse the xml file looking for the config files of each components in the local machine and look for inconsistencies between the deployment file and the config files of each component.
It's not uncommon to have different instances of the same component running in different ports, but one port can only be used by one interface at the same execution time. This tool help the Robocomp developer to find possible inconsistencies. 
Currently it only can check the configuration of local components. 

## How can we execute the `rcdeploymentchecker` tool?

Executing the `rcdeploymentchecker` without parameters or the `-h` option show the full command help:
```shell
rcdeploymentchecker -h
```
```console
usage: rcdeploymentchecker [-h] [-v] [-p PORT] [-a] [-l]
                        {ports,interfaces} [path]

Application to look for existing configured interfaces ports on components

positional arguments:
  {ports,interfaces}    Show the interfaces by name or by port
  path                  path to look for components config files recursively
                        (default="~/robocomp/")

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         increase output verbosity
  -p PORT, --port PORT  List only the selected port information
  -a, --all             show all ports configured for an interface instead of
                        showing only those with more than one interface per
                        port
  -l, --lower           show all ports with numbers lower than 10000
```
## Command examples
- List the ports used by more than one interface
```
        ./rcdeploymentchecker path_to_deployment_xml/
```
The output of the command can show 3 types of lines:

[+] will me shown only if -v option is set.
[?] need to be checked manually. Usually it show some name mismatches (which maybe or maybe not an error).
[!] something is not found and propably need to be fixed.

## TODO
This tools is currently under development and can contain bugs and some information may be missing from the output.
- Remote components can't be checked. 



