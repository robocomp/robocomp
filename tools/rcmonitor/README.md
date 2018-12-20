rcmonitor
===============================

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

##What the rcmonitor does?

The `rcmonitor` is a very useful tool for controlling motors of the robot that we have running in the `rcinnermodel` tool (ideal and perfect model of the robot) or in the real world (the real robot). This tool load all the motors that are working in a determined port, and shows them inside its user interface with their current angular value. We can change the angular value of a motor by selecting it from the list of motors, entering the new value in the spin box below and clicking on the `Set position` button. Also, we can change the speed at which the motor moves and stop its movement with the `stop` button to prevent collisions.

![Alt text](https://github.com/robocomp/robocomp/blob/stable/tools/rcmonitor/examples/rcmonitor.png)

The user interface also allows us to enable or disable the motor, so that when we run a particular simulation, the motors that are disabled can not move.

##How can we execute the rcmonitor tool?

Despite there are many test files into the `example` folder, the most commonly used way to run the rcmonitor is:

1. You must be into the `rcmonitor` folder
2. You must have motors operating. Otherwise the connection between the rcmonitor and the motors will fail.
3. You have to execute the next command:
    
    rcmonitor examples/jointMotorSimple.rcm [-h host_where_motors_are] -p port_where_motors_are_executed

The option `-h` allows us to connect the rcmonitor with motors that are up in another machine (like a robot) and the option `-p`indicates the port where motors are being executed. It is very important put the correct port in the previous sentence, because if we write the port wrong the rcmonitor will give a connection error like this:

    robolab@robolab:~/robocomp/tools/rcmonitor$ rcmonitor examples/jointMotorSimple.rcm -h host -p wrong_port
    Gtk-Message: Failed to load module "overlay-scrollbar"
    Opening configuration file examples/jointMotorSimple.rcm
    [2, 3, 4]
    FILE: examples/jointMotorSimple.rcm
    ROOT: examples
    Opening code file <examples/jointMotorSimple.py>
    Slice options:  -I/home/robocomp/robocomp/interfaces/  -I.  -I/home/robocomp/robocomp/interfaces  --all
    Loading SLICE file: " -I/home/robocomp/robocomp/interfaces/  -I.  -I/home/robocomp/robocomp/interfaces  --all /home/robocomp/robocomp/interfaces/JointMotor.ice"
    1
    2
    3  -I/home/robocomp/robocomp/interfaces/  -I.  -I/home/robocomp/robocomp/interfaces  --all /home/robocomp/robocomp/interfaces/JointMotor.ice
    4
    New module: RoboCompJointMotor
    New module: Ice.Instrumentation
    New module: IceMX
    Done loading /home/robocomp/robocomp/interfaces/JointMotor.ice.
    Traceback (most recent call last):
       File "/opt/robocomp/bin/rcmonitor", line 469, in doJob
             self.doer = self.openA.module.C(self.openA.endpoint, self.openA.sr.RoboComps)
       File "<string>", line 33, in __init__
       File "/home/robocomp/robocomp/interfaces/JointMotor.ice", line 602, in checkedCast
    
    Ice.ConnectionRefusedException: Ice.ConnectionRefusedException:
    Connection refused
    
Normally, the three components that use motors are the commonjoint component (it ups the motors in the port 20000), the faulhaber component (it ups the arm motors in the port 10067) and yhe dynamixel component (it ups the head motors in the port 10068). If we've done everything right, in the rcmonitor user interface appear to us the names of the motors raised in the port. 
