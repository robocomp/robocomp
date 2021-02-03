# Tutorial to start with DSR configurations

This tutorial will guide you through the process of deploying and extending DSR configurations (DSRc). DSRc are sets of agents (a kind of RoboComp component) that share a distributed graph (aka G) data structure. See [robocomp-dsr](https://github.com/robocomp/robocomp/tree/development/classes/dsr") for a detailed description, the theory behind DSR and the installation instructions. 

To have a minimal set of agents acting as the running brain of a (simulated) robot we need four elements:

1. A Robotics simulator such as CoppeliaSim, https://www.coppeliarobotics.com/ that provides us with a realistic physics-based world inhabited by a mobile robot.
2. A Python program (adapter) that starts and runs the CoppeliaSim loop and provides access to all elements in the simulated world, including all the robot's sensors and actuators.
3. A mandatory and already existing agent "idserver" that reads a JSON description of the simulated world into G and makes it available to all other agents that get onboard (C++)
4. A special agent (C++) that connects to the Python adapter through RoboComp interfaces and injects all sensor data into G and also sends all commands to robot.

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
   
In the graph view of the agents, right-click in the laser (203) or camera (210) nodes and select data to open a graphic representation of the sensors. Also, right  clicking on the edges you can see the frame coordinates of the node with respect to its parent or to the world.
   
  ## Creating a brand new agent in 30 seconds with RoboComp's code generator
Now we can move on and create a brand new agent to control de robot. From the situation described before:

 * open a new terminal
 * move to ~/robocomp/components/dsr-graph/components/ and create a new folder "my-agents". 
 * move into "my-agents" and create "my-first-agent". cd into it.
 * execute: robocompdsl my-first-agent.cdsl. A new file will be created with that name
 * open it in your favourite editor and replace the existing code with:

       ``` 
       Component my_first_agent
       { Communications
         {
         };
         language Cpp11;
         gui Qt(QMainWindow);
         options dsr;
       }; 
       
* execute: robocompdsl my-first-agent.cdsl .
* a lot of code will be generated and places into several folders
* build the agent: cmake . ; make; 
* edit the etc/config file and give an id-number to the agent, i.e. 30. *Also set to FALSE the 3d_view flag for now*
* execute it: bin/my-first-agent etc/config

Now you should see a new window with the "good-old" graph view of G. The same G that you can see in the other two agents. It has been copied at start and now the local copy is kept synchronized under the hood by some agent's internal threads.

Now let's write some control code for our Viriato robot. 
 
  
  
  ## A first autonomous driver for the robot
  
  ## Locating objects and people with YOLO, and inserting them in G
  
  
  
