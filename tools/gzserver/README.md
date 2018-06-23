# gazebo-robocomp
The following consists of plugins, tools and libraries for the RoboComp framework to communicate with gazbeo. There are a total of 8 directories which contains different modules of the integration responsible for different funcitonalities.

**gazebo_plugins:** Contains all the gazebo plugins for each major sensor and component used in a robot.
**gazebo_robocomp_models:** Contains gazebo models of the robots and sensors that can be included inside a virtual world and
simulated.<br/>
**gazebo_robocomp_msgs:** Contains all the customized gazebo message types that are used to transfer data between gazebo
plugins and ice interfaces.<br/>
**gazebo_robocomp_worlds:** Contains all scenarios and situations under which a model can be tested.<br/>
**ice-interface:** Contains all ice interfaces that act as relays between the gazebo simulator and robocomp framework.<br/>
**slice:** Contains slice definiton for the interface that are present in the `ice-interface` dir.<br/>
**slice_cpp:** Contains the boiler plate code generated from the slice definitions.<br/>
**src:** Contains utilities to test the gazebo plugins.<br/>

## Installation
To install the repo from source:

```
cd
git clone https://github.com/ksakash/gazebo-robocomp
cd gazebo-robocomp
mkdir build
cd build
cmake .. && make
```

This will the build all the plugins and ice-interfaces. For your system to know where are the plugins and gazebo models, you need to add the location of the built binaries to two gazebo environment variables: `GAZEBO_PLUGIN_PATH` & `GAZEBO_MODEL_PATH`. Add these two lines to your `.bashrc` or `.zshrc`

```
export GAZEBO_PLUGIN_PATH=$HOME/gazebo-robocomp/build/gazebo_plugins
export GAZEBO_MODEL_PATH=$HOME/gazebo-robocomp/gazebo_robocomp_models
```

This will complete the installation.

## Running the integration

In order to use the integration with robocomp framework, for now, you just need to the endpoints at which the `ice-interfaces`, corresponding to all the sensors, are listening. So you can create a component using robocomp framework with the instructions given [here](https://github.com/robocomp/robocomp/blob/master/doc/robocompdsl.md). And, then change the endpoint given in `etc/config` file. So, you can run the gazebo world in which you want to simulate your bot and test your component accordingly.

```
gazebo --verbose ~/gazebo-robocomp/gazebo_robocomp_worlds/<Name-of-the-World>
```

And then you are ready to go!!!
