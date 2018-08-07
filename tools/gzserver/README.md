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
To install the tool `gazeboserver`, you need to just follow the installation guidelines of the framework and it will be installed automatically. To install the framework, from the project's root directory after installing all the dependencies:

```
mkdir build
cd build
cmake ..
make
sudo make install
```

## Checking the installation

At the end of installation one can check the installed robocomp's tools and look from `gazeboserver`. To test the tool, you need to just do:

```
gazeboserver
``` 

If it outputs "Gazebo Server Initialised" on the console, then the tool is installed successfully. One can checkout the video for installation [here](https://youtu.be/sWx-RrONdQM). To use `gazeboserver` you need to just first gazebo and then start `gazeboserver`. Then change the endpoints of the interfaces in your component with the corresponding interfaces' endpoints in `gazeboserver`. Then you are good to go. It is even possible to use `gazeboserver` separately without even integrating it with the main code base which has been decribed [here](https://github.com/ksakash/gazebo-robocomp/blob/master/README.md).
