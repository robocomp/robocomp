RCManager
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What does the RCManager do?

The `rcmanager` is a very useful tool for controlling the components that we have running within RoboComp. `rcmanager`is basically an user interface that shows you a graph that relates RoboComp components with which you are working (nodes of the graph) and relationships or dependencies between them (arrows of the graph).

Let's see the following image:

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/Graph.jpg)

As we can see in it, there are our components that are working (red and green nodes) and dependencies between them ( arrows).

1. NODES: nodes (components) have two possible states:
    > DOWN: This means the components are not working currently.The square in the node will be red in color if the component is down.
    > UP: This means the component is working properly.The square in the node will be green in color if the component is up.
2. ARROWS: they are the dependencies between nodes. The sense of dependence is equal to the direction of the arrow. 

We have another feature to dynamically edit the component tree(Xml file).After editing the file click on the refresh button on the left top of the page.It will update the graph directly. 

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/Editor.jpg)

*   There two additional buttons near the refresh button.The Blue icon button is for adding a template in network setting component in xml edit.The second one with '+' symbol is for adding a template for node in xml edit.The main purpose of these buttons is that we can directly build a full level tree from the scratch.

![Alt text](https://github.com/robocomp/robocomp/tree/highlyunstable/tools/rcmanager/share/rcmanager/XmlButtons.jpg)
      

*   We can edit the settings regarding the xml editor by clicking on the settings button after those two buttons.

## Software model

![Alt text](https://github.com/Kmayankkr/robocomp/blob/highlyunstable/tools/rcmanager/mvc.jpg "")

Reference - [Sencha Touch - MVC](https://www.tutorialspoint.com/sencha_touch/sencha_touch_mvc_explanation.htm)

## How can we execute the rcmanager tool?

Normally we need to be running the [`rcremoteserver`](https://github.com/robocomp/robocomp/tree/master/tools/rcremote)  tool first. Its mainly because the components are usually run through the rcremote tool which acts like a proxy for server tool.

We can either run an already built xml file in rcmanager..Or you can built an entire tree through rcmanager.The advantage of the second one is you can debug the xml file dynamically.


If you are running an already built xml file run the command ..

    rcmanager pathToFile logfileName
or 

    rcmanager pathToFile

Otherwise you can open the file after opening the tool by the command.
    
    rcmanager 

or  

    rcmanager logFileName

##Sample XML File
    
  
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
