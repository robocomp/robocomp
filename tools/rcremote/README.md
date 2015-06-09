[rcremote and rcremoteserver]
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

#How these two components work?

To understand how this component works, we need to have very clear the client-server structure. For example, suppose we have a robot (with his own computer) and our computer, and the robot acts as a server and our computer as a client that needs to access to the robot's services (applications). In this case, the server program (rcremoteserver) will be runned into the robot and the client program (rcremote) will be executed into our computer.
The behaviour of both programs is very similar: both use a default port (4242), an IP address and a password, and work with TCP protocol...

1) rcremoteserver: the server uses its localhost as IP and takes the password that is passed as an input parameter to allow clients to connect or not --> Usage: rcremoteserver password

2) rcremote: the client uses the server IP and the server password to connect with him --> Usage example: rcremote IP_address mytabname /home/robocomp touch argument1 argument2

###File .rcremote

In order to facilitate the use of these two programs we can use the file .rcremote. This file allows us to use the rcremoteserver and rcremote programs without pass arguments. We just have to create the .rcremote file in the HOME directory and follow the format hostname#password, for example:

    robot1#passwordrobot1
    robot2#passwordrobot2
    computer1#passwordcomputer1
    computer2#passwordcomputer2
    [...]
    
This way you only need to run "rcremote" on the client and "rcremoteserver" on the server.



    
    
    



