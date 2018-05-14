##rcinnerModelEditor

This component is used as a tool for changing our innermodel files in graphical way, providing ease to user.


## Configuration parameters

As any other component,
``` *rcinnerModelEditor* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```cp -r ~/robocomp/tools/rcinnerModelEditor ~/robocomp/components ```

First switch to our new rcinnerModelEditor in component directory.

```cd ~/robocomp/components/rcinnerModelEditor```

Then build and run-

```cmake .
sudo make
bin/rcinnerModelEditor --Ice.Config=etc/config```
