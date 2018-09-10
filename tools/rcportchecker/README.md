rcportchecker
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What the `rcportchecker` does?

The `rcportchecker` is a tool for checking the used and availables ports for Robocomp components. This tool looks for the config files of the components in the local machine and show a list of configured components and it ports or a list of ports and the associated components.
This tool can also be used to check if a particular port is available or used in any config file.
It's not uncommon to have different instances of the same components running in different port, but one port can only be used by a component at the same execution time. This tool help the Robocomp developer to find available ports and also to find some posible conflicts when executing several components.

## How can we execute the `rcportchecker` tool?

Executing the `rcportchecker` without parameters or the `-h` option the full command help is shown:
```shell
rcportchecker.py -h
```
```console
usage: rcportchecker.py [-h] [-v] [-p PORT] [-a] [-l] {ports,comps} [path]

Application to look for existing configured ports on components

positional arguments:
  {ports,comps}         Show the components by name or by port
  path                  path to look for components config files recursively
                        (default="~/robocomp/")

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         increase output verbosity
  -p PORT, --port PORT  List only the selected port information
  -a, --all             show all ports configured in a component instead of
                        showing only those with more than one component per
                        port
  -l, --lower           show all ports with numbers lower than 10000
```
## Command examples
- List the ports used in more than one component
```
        rcportchecker.py ports
```
- List all the ports with one or more components configured on this:
```
        rcportchecker.py -a ports
```
- List all the ports with one or more components configured on this and with a number under 10000 (de facto standard is using ports above 10000):
```
        rcportchecker.py -a -l ports
```
- List components configured at the 11003 port if any:
```
        rcportchecker.py -p 11003 ports
```
- List components by name showing the ports used and the path to the config files where it can be found:
```
         ./rcportchecker.py comps
```
## TODO
This tools is currently under development and can contain bugs and some information may be missing from the output.
- The content of files/components-port.txt could be used to check if the ports used ar the default or not.
- Implement a new mode to give the next available port (maybe it could find gaps)
- Create a better hierarchy of arguments with subcommands and parameter groups: https://chase-seibert.github.io/blog/2014/03/21/python-multilevel-argparse.html

