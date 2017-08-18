rcmanager
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

##What the rcmanager does?

The `rcmanager` is a very useful tool for controlling the components that we have running within ROBOCOMP. `rcmanager` is basically an user interface that shows you a graph that relates Robocomp components with which you are working (nodes of the graph) and relationships or dependencies between them (arrows of the graph).

Let's see the following image:

<<<<<<< HEAD
![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/Graph.jpg)
=======
![Alt text](https://github.com/robocomp/robocomp/blob/master/tools/rcmanager/share/rcmanager/rcmanager1.png)
>>>>>>> master

As we can see in it, there are our components that are working (red and green nodes) and dependencies between them (blue arrows).

1. NODES: nodes (components) have two possible states:
    > DOWN: when the nodes are red. This means that the component is not being executed. If we want to run the component, we must click on the node with the right mouse button and select the UP option.
    > UP: when the nodes are green. This means that the component is being executed. If we want to stop the component, we have to click on the node with the right mouse button and select the DOWN option.
2. ARROWS: they are the dependencies between nodes. The sense of dependence is equal to the direction of the arrow. For example, in the first image we can see that the commonjoint component (which is responsible for handling all robot motors) depends on two other components: the dynamixel component (which is responsible for handling the head motors) and the faulhaber component (which is responsible for handling the arm motors).

Another view offered by this program is what we see in the picture below:

<<<<<<< HEAD
![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/Editor.jpg)

*   There two additional buttons near the refresh button.The Blue icon button is for adding a template in network setting component in xml edit.The second one with '+' symbol is for adding a template for node in xml edit.The main purpose of these buttons is that we can directly build a full level tree from the scratch.

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/XmlButtons.jpg)
      

*   We can edit the settings regarding the xml editor by clicking on the settings button after those two buttons.
=======
![Alt text](https://github.com/robocomp/robocomp/blob/master/tools/rcmanager/share/rcmanager/rcmanager2.png)

*   In the first box we have the components names in red (not running) and green (running)
*   Next, if we select one component (like the commonjoint) we can down the execution of the component (and we can see where the component is being executed, in this case the commonjoint component is being executed inside the nuc1 of the robot Ursus). Also, we can see what is its configuration file and the port where it is being executed.
*   In the last box we have a log with the history of actions taken. 
>>>>>>> master

##How can we execute the rcmanager tool?

Normally we need to be running the `rcremoteserver` and the `rcremote` tools first. Then we have to create an .xml file that indicates the components that we want to appear in the rcmanager. Let see the next example:

    ```xml```
    <?xml version="1.0" encoding="UTF-8"?>
    
    <rcmanager>
       <generalInformation>                                    <!--SOME CONFIGURATIONS -->
          <editor path="kate" dock="false" />
          <timeouts fixed="1000.0" blink="300.0" />
          <clicks switch="2.0" interval="400.0" />
          <graph alpha="80.0" active="true" scale="200.0" />
          <graphTiming idletime="1000.0" focustime="500.0" fasttime="10.0" fastperiod="2000.0" />
          <simulation hookes="0.07" springlength="0.5" friction="0.4" step="0.5" fieldforce="20000.0" />
       </generalInformation>
       
       [...]
       
       <!-- THE COMMON JOINT COMPONENT -->
       <node alias="commonjoint" endpoint="jointmotor:tcp -h robonuc1.local -p 20000">           <!-- WHERE THE COMPONENT IS -->
          <dependence alias="faulhaber" />                                                       <!-- DEPENDENCE WITH THE FAULHABER COMPONENT -->
          <dependence alias="dynamixel" />                                                       <!-- DEPENDENCE WITH THE DYNAMIXEL COMPONENT -->
          <workingDir path="/home/robocomp/" />                                                  <!-- THE WORKING DIRECTORY -->
          <upCommand command="rcremote robonuc1.local commonjoint /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/ ./bin/ursuscommonjointcomp 
          --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursusCommon.conf"/> <!-- HOW WE START THE COMPONENT -->
          <downCommand command="ssh robolab@robonuc1.local killall -9 ursuscommonjointcomp" />   <!-- HOW WE STOP THE COMPONENT -->
          <configFile path="/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.conf" /> <!-- WHERE THE CONFIG FILE IS -->
          <!-- TO DRAW THE NODE INTO THE RCMANAGER UI -->
          <xpos value="116.463101156" />
          <ypos value="20.0105420209" />
          <radius value="10.0" />
       </node>
       
       <!-- THE FAULHABER COMPONENT -->
       <node alias="faulhaber" endpoint="jointmotor:tcp -h robonuc1.local -p 10067">              <!-- WHERE THE COMPONENT IS -->
          <workingDir path="/home/robocomp/" />                                                   <!-- THE WORKING DIRECTORY -->
          <upCommand command="rcremote robonuc1.local faulhaber /home/robocomp/robocomp/components/robocomp-ursus/components/faulhaberComp/bin/ ./faulhaberComp 
          --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/faulhaber.conf" />   <!-- HOW WE START THE COMPONENT -->
          <downCommand command="ssh robolab@robonuc1.local killall -9 faulhaberComp" />           <!-- HOW WE STOP THE COMPONENT -->
          <configFile path="/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.conf" />  <!-- WHERE THE CONFIG FILE IS -->
          <!-- TO DRAW THE NODE INTO THE RCMANAGER UI -->
          <xpos value="179.660065085" />
          <ypos value="98.1737308868" />
          <radius value="10.0" />
       </node>
    
<<<<<<< HEAD
  
        ```xml```
      <?xml version="1.0" encoding="UTF-8"?>

      <rcmanager>

        <generalInformation>  
          <group name="group" iconfile="/home/h20/Coding                          /Robocomp/GSOC_TOOL/rcmanager/rcmanager/share/rcmanager/1465394415_floppy.png" />  <!--This is the way to manually define a group (The iconfile is mandatory)-->

        </generalInformation>

        <node alias="SIM" endpoint="differentialrobot:tcp -h localhost -p 10004"> <!-- WHERE THE COMPONENT IS.To check the status of component -->
          <dependence alias="Controller" />
          <group name="group"/>                                                   <!--We add This componet To the group-->
          <workingDir path="/home/h20/robocomp/files/innermodel" />               <!-- THE WORKING DIRECTORY -->
          <upCommand command="rcremote 127.0.0.1 SIM /home/h20/robocomp/files/innermodel rcis simpleworld.xml" /> <!-- HOW WE START THE COMPONENT -->
          <downCommand command="pkill -9 -f rcis" />                              <!-- HOW WE STOP THE COMPONENT -->
          <configFile path="" />                                                  <!-- WHERE THE CONFIG FILE IS -->
          <xpos value="0.0" />
          <ypos value="0.0" />
          <ip value="127.0.0.12"/>   <!--An optional Value .If not given tool will consider as local host.The component on same Ip will have same background Color for Icons-->  
        </node>

        <node alias="Controller" endpoint="CommonBehavior.Endpoints:tcp -p 1">
          
          <workingDir path="/home/robocomp/" />
          <upCommand command="rcremote 127.0.0.1 controller /home/robocomp/robocomp/components/robocomp-robolab/components/keyboardrobotcontroller/src/ /home/robocomp/robocomp/components/robocomp-robolab/components/keyboardrobotcontroller/src/keyboardrobotcontroller.py" />
          <downCommand command="pkill -9 -f keyboardrobotcontroller.py" />
          <configFile path="/etc/config" />
          <xpos value="200.0" />
          <ypos value="-200.0" />
          <ip value="127.0.0.1"/>
        </node>

      </rcmanager>


##Extra Features in the new tool

All the Networks build for the old tool is compatible with the new rcmanager.The values like group will be taken default values if they are not mentioned in the xml file..


All the components in the same group will be having a same icon..If the component does not belong to any group.It will be have a default icon.

All group Related  functinalities like adding to group,Deleting from group..Uping the group can be selected from right clicking on the component and selecting option from Group menu.

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/GroupSelector.jpg)


New groups can be created dynamically by.

      rightClick on background > NewGroup


![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/GroupBuilder.jpg)


New component can be added either by right clicking on background and selecting 'New Component' or clicking on the '+' icon button on right top side..If you click the button the new component Will be created on origin.

There is a optional data of 'ip address'.If not given tool will take it as local host.

There is a new option for logging data into file.You can either set the logFile by.

    tools>Set LogFile

##Or 
    
Enter the logFile name along with the tool starting command.The log filename should endwith .logging

##Using the xml file build for old tool.

We have made the new tool backward compatible with the old one.So all trees build on the old one should be working fine.The componets will look much closer(infact crowded) in the new tool.It is because of the changed size of nodes..All you have to do is.


    rightClick on background > Graph >Stretch

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/GraphStretch.jpg)

it will multiply the position with stretch factor and update the xml file..

For opening and saving files usual shortcuts will be working..

You can edit the xml settings.

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/XmlEditorSettings.jpg)


Searching the component is also possible by typing the name of component on right top search area..
=======
       [...]
    </rcmanager>  
    
Now, to execute the rcmanager, we only put in the command line `rcmanager manager.xml` and that's it!!

>>>>>>> master
