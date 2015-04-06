#AprilTagsComp: wrapping AprilTags in RoboComp

AprilTags is an augmented reality tag system developed by E. Olson at U. of Michigan, USA. A complete explanation and related papers can be found [here](http://april.eecs.umich.edu/wiki/index.php/AprilTags). There is a C++ version written by Michael Kaes [here](http://people.csail.mit.edu/kaess/apriltags/) which is the one we use.

April tags are AR tags designed to be esily detected by (robot) cameras. When the tag is seen by the camera, the algorithm computes the tag's complete pose defining its own reference system relative to the camera. This reference system is defined as follows: if we look perpendicularly to a non rotated tag, the Z+ axis comes out towards us from the center of the tag plane, the X+ axis points leftwards and the Y+ axis points upwards (a left-hand reference system). The values computed by *apriltagsComp* are the translation vector from the camera to the center of the tag's reference system, and the three Euler angles that encode the relative orientation of the tag's reference system wrt to the camera reference system.

The *AprilTags.cdsl* file specifies how *apriltagsComp* has been generated and how it can be re-generated:

    import "/robocomp/interfaces/IDSLs/GetAprilTags.idsl";
    import "/robocomp/interfaces/IDSLs/AprilTags.idsl";
    import "/robocomp/interfaces/IDSLs/RGBD.idsl";
    import "/robocomp/interfaces/IDSLs/RGBDBus.idsl";
    import "/robocomp/interfaces/IDSLs/Camera.idsl";
    Component AprilTagsComp{
        Communications{
                requires Camera, RGBDBus, RGBD;
                publishes AprilTags;
                implements GetAprilTags;
        };
        language Cpp;
    };

This files tells us that the component requires -will be calling- three RoboComp interfaces: Camera, RGBDBus y RGBD, which are normal and depth camera's interfaces written in RoboComp's IDSL language. You can find those files in *~/robocomp/interfaces/IDSLs*. Also, the component will publish the data defined in the *AprilTags* interface and will implement the *GetAprilTags* interface. This means that using images provided by a component implementing the camera or RGBD interfaces, it will try to detect any tags in them and compute their 6D pose. Finally, it will publish a vector with all the tags id's and poses to the Ice's STORM broker, and also it will attend any direct requests (remote procedure calls) received from other components through the *GetAprilTags* interface. So it is a rather serviceable and handy component!

To access **apriltagsComp** you need to install from *http://github.org/robocomp* the repository named *robocomp-robolab*. 

    cd ~/robocomp/components
    git clone https://github.com/robocomp/robocomp-robolab.git

Once downloaded, *apriltagsComp* can be found in:

    ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    
First, read the *README.TXT* file and follow instructions thereby. Once the library has been installed in /usr/local, we can proceed to compile the component:

    cd ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    cmake .
    make
    
We should have a binary now:

    ~/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp
    

##Configuration parameters
As any other component, *apriltagsComp* needs a *config* file to start. 

In

    ~/robocomp/components/robocomp-robolab/components/apriltagsComp/etc/generic_config
   
you can find an example of a configuration file. We can find there the following lines:


##Starting the component
To start the component we need whether a real camera connected to the cameraV4lComp component or the RCIS simulator started with a file that includes virtual tags, such as *simpleworld.xml*. 

Once RCIS is up and running, it will attend the 
    


    





