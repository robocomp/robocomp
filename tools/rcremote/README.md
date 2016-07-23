rcremote and rcremoteserver

Managing and monitoring the execution of the component network in a robot can be time-consuming when the number of components grows. To mitigate this problem, RoboComp provides the rcmanager tool. After defining which commands should be used to run each of the components and their dependencies, rcmanager can be easily used to manage a component network. Despite it is extremely useful, there are still two limitations to overcome: a) components using graphics output can not be directly run through ssh, which is an important issue when robots have multiple computers onboard b) RCManager does not show the textual output of the components which have been executed. The rcremote tools were designed to overcome these limitations.

The rcremote tool set is composed of two tools: rcremoteserver and rcremote, server and client respectively. The server is executed in each of the computers the robot has and the client should be invoked by rcmanager. When a command is executed remotely using rcremote, the server uses a yakuake tab for each of the executed components, so the developers can see the textual output. Since the server is executed locally, there are no limitations regarding graphic output.

#Configuration
The configuration file '.rcremote' should be placed in each of the home directories of the users in the robot's hosts and the computer which is going to be used to run the components. The format is the following:

 host1#password1
 host2#password2
 host3#password3

The passwords are sent hashed along with a seed, but the file format is plain, so make sure other users cannot read the files.


#Use
The command to remotely run components, which can be executed manually, in a script or using rcmanager has the following parameters:

 rcremote host tabname pwd binary [ parameter1 [parameter2 [...] ] ]

Example:

 rcremote robotHost1.local rgbd /home/robocomp rgbdComp etc/rgbd.conf


    
    



