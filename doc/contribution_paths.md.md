# Contribution paths

## Contributing to DSR core

## Contributing to DSR agents
We are currently working with the main branch of robocomp.
If you have installed any other version of robocomp it's recomended to clena the previous installation:
```bash
sudo rm -r /usr/local/lib/cmake/dsr_*
sudo rm -r /usr/local/lib/cmake/qmat/
sudo rm -r /usr/local/lib/libdsr_*
sudo rm -r /opt/robocomp/
```

Then set your robocomp branch to main, remove the build directory if it already exists and go with the configuration, compilation and installation:
```bash
mkdir build; cd build; cmake -DDSR=TRUE -DFCL_SUPPORT=TRUE .. ; make clean ; make -j10; sudo make install
```

In the main brach of robocomp, DSR needs to be installed separatly:
```
cd classes/dsr/; mkdir build ; cmake .. ; make clean ; make -j10; sudo make install
```

Now you can download the robocomop/dsr-graph repository:
cd components
```bash
git clone https://github.com/robocomp/dsr-graph.git
```

In the components folder of this repository you can find some of the esential agents you would need in mostly any DSR deplyment.

Also in robots_pyrep you will find some of the virtualizations of robots created to be used with Coppelia SIM.

At this point, if you are still not involved in the DSR development, you would probably want to follow this tutorial:
https://github.com/robocomp/robocomp/blob/development/doc/DSR-start.md

> Written with [StackEdit](https://stackedit.io/).


> Written with [StackEdit](https://stackedit.io/).
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTczMzY1NjgzOF19
-->