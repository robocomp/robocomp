
# Build tools

Note: This documentation explains some commands that can be used for the development of components in Robocomp. These commands are not intended to compile Robocomp itself. To follow the necessary steps to install and compile Robocomp, you can follow the instructions in this [document](https://github.com/robocomp/robocomp#installation-from-source).

Following are a list of commands and explanations for Robocomp build tools.
## rcworkspace

This tool helps in managing robocomp workspaces. 

#### To register a new workspace

Move to a directory that contains robocomp components

Use the -a flag to register the workspace and begin component searching within it.
```bash
rcworkspace -a 
```

**The build tools will find only those workspaces which are registered on your computer**.
    
```bash
rcworkspace [-h] [-i INITIALIZE] [-a [ADD]] [--accept-all] [-u]
                   [-d [DELETE]] [--clear-all] [-l]
                   [directory]
```
For  more uses of rcworkspace use the -h flag.
## rcbuild

When invoked from a workspace, without any arguments, if not inside the source path, it will build all the non-ignored components inside the workspace. 

If inside a component source directory, it will build only that component. 

If a component is specified it will build only that component. You can find the build targets in the devel space.
    
```bash
rcbuild [-h] [-i [INSTALL] | --doc | --installdoc] [component]
```

The `doc` will generate documentation, `installdoc` will install the docs to install path, `install` will build and install the components to the specified directory. 

If nothing is specified it will be installed to `/opt/robocomp`. Currently, you can only generate docs for one component at a time.

## rccomp

It lists either the registered components or workspaces.

```bash
rccomp [-h] [{list,listws}]
```
 
`rccomp list` will list all the components in registered workspaces and `rccomp listws` will list all the registered workspaces.

## rced

When invoked as component-name with file-name. it will open the file in the component. 

If multiple files with the same name exist, it will give choices and ask you to choose one. It uses the editor specified in `$EDITOR` (like nano) by default, if not present it will use `vim`.

```bash
 rced [-h] component file
```

## rcrun

Using rcrun you can start, stop or force stop any component from anywhere. 

You can also start a component in debug mode, given you have the required *config file* in the */etc* directory. If you have specified a config file then rcrun will use it to start the component. 

By default, rcrun will use the `config` config file in `etc` directory, if not found it will search for `generic_config`. If not found it will use any of the config files present.

If the debug flag is set, it will search for a config file that ends with *.debug*.

```bash
rcrun [-h] [-s START |-st STOP | -fst FSTOP] [-d | -cf CFILE | -c CONFIG] [-is] [component]
```

## rbcd

Using this you can cd into the component directory given the component name.

```bash
rbcd component
```

## Notes

* The build tools don't support python components yet.
* Auto-completion is enabled all the components. It works by pressing tab when you have written enough letters, so the auto-completion knows what words you are about to write.
* In case you run into issues running any build tool, kindly report it.
