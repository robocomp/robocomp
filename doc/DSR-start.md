# Tutorial to start with DSR configurations

This tutorial will guide you through the process of deploying and extending DSR configurations (DSRc). DSRc are sets of agents (a kind of RoboComp component) that share a distributed graph (aka G) data structure. See [robocomp-dsr](https://github.com/robocomp/robocomp/tree/development/classes/dsr") for a detailed description, the theory behind DSR and the installation instructions. 

To have a minimal set of agents acting as the running brain of a (simulated) robot we need four elements:

1. A Robotics simulator such as CoppeliaSim, https://www.coppeliarobotics.com/ that provides us with a realistic physics-based world inhabited by a mobile robot.
2. A Python program (adapter) that starts and runs the CoppeliaSim loop and provides access to all elements in the simulated world, including all the robot's sensors and actuators.
3. A mandatory and already existing agent "idserver" that reads a JSON description of the simulated world into G and makes it available to all other agents that get onboard (C++)
4. A special agent (C++) that connects to the Python adapter through RoboComp interfaces and injects all sensor data into G and also sends all commands to robot.

__NOTE :__ Refer to these [common issues](https://github.com/robocomp/grasping/blob/master/DSR-INTEGRATION.md#common-issues) for solutions to previously-known issues while installing and running DSR agents.

## DSR's "Hello World"
Here we will describe the steps required to start all four elements. With the addition of a joystick or a XBox pad, you can move the robot around and see all the nice tools included in G and the G viewers that all agents inherit.

Once you have completed the installation of robocomp, robocomp-robolab (hardware robot drivers) and  [robocomp-dsr](https://github.com/robocomp/dsr-graph "robocomp-dsr"), follow these steps: 

_(we are assuming that your robocomp repo is in ~/robocomp/ and that you have clones robocomp-robolab and dsr-graph below ~/robocomp/components)_
#### Attention: it is crucial that you uninstall with pip3 the package _python3-opencv_ and install the _python3-opencv-headless_ version

1. In a new terminal type "rcnode" to start ZeroC's publish-subscribe broker
2. In a new terminal goto ~/robocomp/components/dsr-graph/robots_pyrep/viriatoPyrep
 * Build it with cmake and run it using the provided "run.sh" script. 
3. In a new terminal goto ~/robocomp/components/dsr-graph/components/idserver. 
  * Build it and execute with _bin/idserver etc/config_ 
  * You should see a Qt window showing the graph G. There are also other tabs that "project" G as a 2D and 3D scene. Also, there is a tree view of G.
4. In a new terminal goto ~/robocomp/components/dsr-graph/components/viriatoDSR and
  * Build it and execute with _bin/viriatoDSR etc/config_
  * On startup, the agent will request a copy of G and upon reception it will show it in its own window. All views of G will be equal since they are copies of a synchronized replicated data structure (similar to a GDoc)
  
 Now we have the simplest DSRc runnning and connected to the simulated world that CoppeliaSim brings to life. Yo need now to get familiarized with the agents' UI and with G. To see how things change when there is movement in the world we have two ways:
 
 1. using a joystick or XBox pad.
   * in a new terminal goto ~/robocomp/components/robocomp-robolab/hardware/external_control/joystickpub
   * build the component and start it with _bin/joystickpub etc/config_
   * move the stick and you will see the robot moving in CoppeliaSim and in ALL G views.
 2. from the CoppeliaSim UI. 
   * click on the Coppelia window and click on the robot (check in the tree view on the left that you have selected the topmost element of the tree -Viriato)
   * click on the button in the upper panel of CoppeliaSim with a little cube in the middle and four arrows around. 
   * Move the mouse dragging the robot around and see how all G views are refreshed.
   
   
  ## Creating a brand new agent in 30 seconds with RoboComp's code generator
  
  ## A first autonomous driver for the robot
  
  ## Locating objects and people with YOLO, and inserting them in G
  
  
  
