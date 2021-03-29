rcmanager
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What the rcmanager does?

The `rcmanager` is a very useful tool for controlling the components that we have running within ROBOCOMP. The `rcmanager` is basically a user interface that shows you a graph that relates the RoboComp components with nodes (of the graph) and relationships/dependencies between them (arrows of the graph).

Let's see the following image:

![Alt text](https://github.com/robocomp/robocomp/blob/stable/tools/rcmanager/share/rcmanager/rcmanager1.png)

As we can see in it, there are our components that are working (red and green nodes) and dependencies between them (blue arrows).

1. NODES: nodes (components) have two possible states:
    > DOWN: when the nodes are red. This means that the component is not being executed. If we want to run the component, we must click on the node with the right mouse button and select the UP option.
    > UP: when the nodes are green. This means that the component is being executed. If we want to stop the component, we have to click on the node with the right mouse button and select the DOWN option.
2. ARROWS: they are the dependencies between nodes. The direction of the arrow represents what depends on what. For example, in the first image, we can see that the commonjoint component (which is responsible for handling all robot motors) depends on two other components: the dynamixel component (which is responsible for handling the head motors) and the Faulhaber component (which is responsible for handling the arm motors).

Another view offered by this program is what we see in the picture below:

![Alt text](https://github.com/robocomp/robocomp/blob/stable/tools/rcmanager/share/rcmanager/rcmanager2.png)

*   In the first box we have the components names in red (not running) and green (running)
*   Next, if we select one component (like the commonjoint) we can down the execution of the component (and we can see where the component is being executed, in this case the commonjoint component is being executed inside the nuc1 of the robot Ursus). Also, we can see what is its configuration file and the port where it is being executed.
*   In the last box we have a log with the history of actions taken. 

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
    
       [...]
    </rcmanager>  
    
Now, to execute the rcmanager, we only put in the command line `rcmanagersimple manager.xml` and that's it!!
