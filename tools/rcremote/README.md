rcremote and rcremoteserver
===============================

Managing and monitoring the execution of the component network in a robot can be time-consuming when the number of components grows. To mitigate this problem, RoboComp provides the rcmanager tool. After defining which commands should be used to run each of the components and their dependencies, rcmanager can be easily used to manage a component network. Despite it is extremely useful, there are still two limitations to overcome: a) components using graphics output can not be directly run through ssh, which is an important issue when robots have multiple computers onboard b) RCManager does not show the textual output of the components which have been executed. The rcremote tools were designed to overcome these limitations.

The rcremote tool set is composed of two tools: rcremoteserver and rcremote, server and client respectively. The server is executed in each of the computers the robot has and the client should be invoked by rcmanager. When a command is executed remotely using rcremote, the server uses a yakuake tab for each of the executed components, so the developers can see the textual output. Since the server is executed locally, there are no limitations regarding graphic output.

#Configuration

#Use


1) rcremoteserver: the server uses its localhost as IP and takes the password that is passed as an input parameter to allow clients to connect or not --> Usage: `rcremoteserver password`

2) rcremote: the client uses the server IP and the server password to connect with him --> Usage example: `rcremote IP_address mytabname /home/robocomp touch argument1 argument2`

###File .rcremote

What happends if we forget to put the password when we run the rcremoteserver or the rcremote program? An error like this will be released:
    
    IOError: [Errno 2] No existe el archivo o el directorio: '/home/<your-linux-user>/.rcremote'

In order to prevet this and facilitate the use of these two programs we can use the file `.rcremote`. This file allows us to use the rcremoteserver and rcremote programs without pass arguments. We just have to create the `.rcremote` file in the HOME directory and follow the format `hostname#password`, for example:

    robot1#passwordrobot1
    robot2#passwordrobot2
    computer1#passwordcomputer1
    computer2#passwordcomputer2
    [...]
    
This way you only need to run "rcremote" on the client and "rcremoteserver" on the server.



    
    
    



