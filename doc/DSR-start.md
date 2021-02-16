# Tutorial to start with DSR configurations

This tutorial will guide you through the process of deploying and extending DSR configurations (DSRc). DSRc are sets of agents (a kind of RoboComp component) that share a distributed graph (aka G) data structure. See [robocomp-dsr](https://github.com/robocomp/robocomp/tree/development/classes/dsr) for a detailed description, the theory behind DSR and the installation instructions. 

To have a minimal set of agents acting as the running brain of a (simulated) robot we need four elements:

1. A Robotics simulator such as CoppeliaSim, https://www.coppeliarobotics.com/ that provides us with a realistic physics-based world inhabited by a mobile robot.
2. A Python program (adapter) that starts and runs the CoppeliaSim loop and provides access to all elements in the simulated world, including all the robot's sensors and actuators.
3. A mandatory and already existing agent "idserver" that reads a JSON description of the simulated world into G and makes it available to all other agents that get onboard (C++)
4. A special agent (C++) that connects to the Python adapter through RoboComp interfaces and injects all sensor data into G and also sends all commands to robot.

## DSR's "Hello World"
Here we will describe the steps required to start all four elements. With the addition of a joystick or a XBox pad, you can move the robot around and see all the nice tools included in G and the G viewers that all agents inherit.

Once you have completed the installation of robocomp, robocomp-robolab (hardware robot drivers) and  [robocomp-dsr](https://github.com/robocomp/dsr-graph), follow these steps: 

_(we are assuming that your robocomp repo is in ~/robocomp/ and that you have clones robocomp-robolab and dsr-graph below ~/robocomp/components)_
#### Attention: it is crucial that you uninstall with pip3 the package _python3-opencv_ and install the _python3-opencv-headless_ version

1. In a new terminal type "rcnode" to start ZeroC's publish-subscribe broker
2. In a new terminal goto ~/robocomp/components/dsr-graph/robots_pyrep/viriatoPyrep
 * Build it with cmake and run it using the provided "run.sh" script. 
3. In a new terminal goto ~/robocomp/components/dsr-graph/components/idserver. This agent will read a stored graph from a file.
  * Build it and execute with _bin/idserver etc/config_ 
  * You should see a Qt window showing the graph G. There are also other tabs that "project" G as a 2D and 3D scene. Also, there is a tree view of G.
4. In a new terminal goto ~/robocomp/components/dsr-graph/components/viriatoDSR. This agent is the connection between viriatoPyrep (simulator or real robot) and G
  * Build it and execute with _bin/viriatoDSR etc/config_
  * On startup, the agent will request a copy of G and upon reception it will show it in its own window. All views of G will be equal since they are copies of a synchronized replicated data structure (similar to a GDoc)
 
 Terminal 1
 ```bash
 rcnode
 ```
 Terminal 2
 ```bash
 cd ~/robocomp/components/dsr-graph/robots_pyrep/viriatoPyrep
 cmake .
 bash run.sh
 ```
 
 Terminal 3
 ```bash
 cd ~/robocomp/components/dsr-graph/components/idserver
 cmake .
 make
 ./bin/idserver etc/config
 ```
 
 Terminal 4
 ```bash
 cd ~/robocomp/components/dsr-graph/components/viriatoDSR
 cmake .
 make .
 ./bin/viriatoDSR etc/config
 ```
 
  
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
Now we can move on and create a brand new agent to control de robot. Remember that viriatoPyrep, idserver and viriatoDSR must be running to proceed with this.

 * open a new terminal
 * move to ~/robocomp/components/dsr-graph/components/ and create a new folder "my-agents". 
 * move into "my-agents" and create "my_first_agent". cd into it.
 * execute: robocompdsl my_first_agent.cdsl. A new file will be created with that name
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
      ```
       
* execute: robocompdsl my_first_agent.cdsl .
* a lot of code will be generated and placed into several folders.
* this is the moment to add the new my-frist-agent folder to your git repo. After we go with cmake, a lot of _garbage_ will be created that you don't want to upload.
* build the agent: cmake . ; make; 
* edit the etc/config file and give an id-number to the agent, i.e. 30. *Also set to FALSE the 3d_view flag for now*
* execute it: bin/my_first_agent etc/config

Now you should see a new window with the "good-old" graph view of G. The same G that you can see in the other two agents. It has been copied at start and now the local copy is kept synchronized under the hood by some agent's internal threads.

**The thirty seconds end here!** Now let's write some control code for our Viriato robot. 

_NOTE_: If you don't understand the next script, don't copy an paste it in your terminal. It's only a reference script with the previous steps. 
```bash
cd ~/robocomp/components/dsr-graph/components/
mkdir my-agents
cd my-agents
mkdir my_first-agent
cd my_first_agent
robocompdsl my_first_agent.cdsl
echo "\
Component my_first_agent
{
  Communications
  {
  };
  language Cpp11;
  gui Qt(QMainWindow);
  options dsr;
};" >  my_first_agent.cdsl
robocompdsl my_first_agent.cdsl .
git add .
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ..
sed -i 's/agent_id = 0/agent_id = 30/g' etc/config
sed -i 's/3d_view = true/3d_view = false/g' etc/config
./bin/my_first_agent etc/config
```



## Modifying your brand new agent

We will add code to SpecificWorker.h and cpp, so let's start with the first one.

Add these two lines to the _include_ section

  ``` 
  #include  "../../../etc/viriato_graph_names.h"
  #include <random>

  ``` 
Now in SpecificWorler.cpp. In the class constructor add this line:
 
 ```
   QLoggingCategory::setFilterRules("*.debug=false\n");
 ```  
 
 and replace the compute() method there with this one. Laser data in Viriato comes from the composition of two 180º LIDARS placed in opposite corners. A 360º representation is built and inserted in G as two float arrays, one with the scanning angles in radians and starting with -M_PI, and another one of the same size with the distances in millimeters. We seek a frontal cone starting in -PI/6 and ending in PI/6, given that the robot's front is 0º.
 
   ```
   void SpecificWorker::compute()
 { 
     const float MIN_DIST = 800.;  // min distance allowed to obstacles
     const float MAX_ROT = 1; // rad/sg
     const FLOAT MAX_ADV = 500;  // mm/sg

     // Here we retrieve data in G. Since G is a distributed data structure shared with other agents, there is no guarante 
     // that the selected nodes will be there. This is why the API returns std::optional<> types.
     // In this case we trust others and directly access its value with value().
     
     // Safely get a local copy of the complete laser_node. It is being updated by the viriatoDSR agent with data from CoppeliaSim. 
     auto laser_node = G->get_node(laser_name).value();
     
     // Define a const reference to the laser_node attribute: _laser_angles_att_ and avoid copying the data.
     const auto &angles = G->get_attrib_by_name<laser_angles_att>(laser_node).value().get();  // from -PI to PI
     
     // Find the index of the first element in angles that is greater than -PI/6
     auto left_limit = std::distance(angles.begin(), std::find_if(angles.begin(),angles.end(), [](auto &a){ return a > -M_PI/6;}));
     
     // Find the index of the first element in angles that is greater than PI/6
     auto right_limit = std::distance(angles.begin(), std::find_if(angles.begin(),angles.end(), [](auto &a){ return a > M_PI/6;}));
     
     // Define a const reference to the laser_node attribute: _laser_dists_att_ and avoid copying the data. 
     const auto &dists = G->get_attrib_by_name<laser_dists_att>(laser_node).value().get();
     
     // Compute the minimum of the dists vector between the computed limits
     auto min_dist = std::min(dists.begin() + left_limit, dists.begin() + right_limit);
     
     // Define the three target speeds for the robot
     float adv = 0; float rot = 0; float side= 0;

     // The final simple logic
     if(*min_dist < MIN_DIST)
     {
         adv = 0; side = 0; rot = MAX_ROT;
     }
     else
     {
         adv = MAX_ADV; side = 0; rot = 0;
     }

     // Write in the robot_node's attributes the desired target speeds for the robot
     G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), adv);
     G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), rot);
     G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), side);
     
     // Write back the node to G so _viriatoDSR_ notices and sends the commands down to the simulator.
     G->update_node(robot_node.value());
 }
 ```
  
Now you can compile again the agent with _make_ and give it a run. It is far from perfect but should give an idea of how a basic agent is built. Now we can improve it and add new agents to keep on building the robot's brain.
  
  
  
