---
layout: post
title: aprilTagsComp, Wrapping E. Olson's AprilTags in RoboComp
categories: [Tutorial]
tags: [AprilTags]
description: AprilTags is an augmented reality tag system developed by E. Olson at the U. of Michigan, USA. A complete explanation and related papers can be ...
---
#aprilTagsComp: wrapping E. Olson's AprilTags in RoboComp

AprilTags is an augmented reality tag system developed by E. Olson at the U. of Michigan, USA. A complete explanation and related papers can be found [here](http://april.eecs.umich.edu/wiki/index.php/AprilTags). There is a C++ version written
 by Michael Kaes [here](http://people.csail.mit.edu/kaess/apriltags/) which is the one we use.

April tags are AR tags designed to be easily detected by (robot) cameras. Understand them as a visual fiducial (artificial features) system that uses a 2D bar code style “tag”, allowing full 6 DOF localization of features from a single image. It is designed to encode smaller data (between 4 and 12 bits) and also these tags can be detected by the camera even at odd conditions. When the tag is seen by the camera, the algorithm computes the tag's complete pose defining its own reference system relative to the camera (i.e Location of the tag is known with high accuracy). This reference system is defined as follows: If we look perpendicularly to a non rotated tag, The Z+ axis comes out towards us from the center of the tag plane, The X+ axis points leftwards and the Y+ axis points upwards (a left-hand reference system). The values computed by *apriltagsComp* are the translation vector from the camera to the center of the tag's reference system, and the three Euler angles that encode the relative orientation of the tag's reference system wrt to the camera reference system.

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
    
First, read the *INSTALL_APRILTAGS_LIB.TXT* file and follow instructions thereby. Once the library has been installed in /usr/local, we can proceed to compile the component:

    cd ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    cmake .
    make
    
We should have a binary now:

    ~/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp
    

##Configuration parameters
As any other component, *apriltagsComp* needs a *config* file to start. In

    ~/robocomp/components/robocomp-robolab/components/apriltagsComp/etc/generic_config
   
you can find an example of a configuration file. We can find there the following lines:

    GetAprilTagsComp.Endpoints=tcp -p 12210                     //Port where GetAprilTags iface is served
    CommonBehavior.Endpoints=tcp -p 11258                       //Not of use for the user now
    CameraProxy = camera:tcp -h localhost -p 10001              //Port where a camera is located
    RGBDProxy = rgbd:tcp -h localhost -p 10096                  //Port where a RGBD camera is located
    RGBDBusProxy = rgbdbus:tcp -h localhost -p 10239            //Port where a bus of RGBDs is located
    AprilTagsProxy = apriltags:tcp -h localhost -p 10261        //Not of use for the user
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999    //Port where STROM broker is located
    InnerModelPath=/home/robocomp/robocomp/files/innermodel/simpleworld.xml


    InputInterface = RGBD                                       //Current input iface to be used
    AprilTagsFamily = tagCodes36h11                             //Tags family. See AprilTags paper
    AprilTagsSize = 0.17                                        //Tag default real size in meters
    ID:0-10 = 0.17   #tag size in meters                        //Tags numbers 1-10 real size in meters
    ID:11-20 = 0.17   #tag size in meters                       //Tags numbers 11-20 real size in meters
    ID:21-30 = 0.17   #tag size in meters                       //Tags numbers 21-30 real size in meters

AprilTagsFamily is a set of tags, There are different families like 36h10,25h9,16h5 however *tagCodes36h11* is recommended. Each tag has an ID that is printed inside the surrounding square using Hamming code. Instructions to print tags and other tag families can be found [here](http://april.eecs.umich.edu/wiki/index.php/AprilTags). The algorithm needs the real size of the tag to estimate its position and orientation in space. We can give the component tags of different sizes, As long as they correspond to different ranges of IDs, as specified in the configuration file above.

##Starting the component
To start the component we need a real camera connected to the cameraV4lComp component or the RCIS simulator started with a file that includes virtual tags, such as *simpleworld.xml*, Tutorial can be found [here](virtualapriltagstutorial.md). Once RCIS is up and running, It will provide the RGBD.idsl interface (not Camera.idsl for now) at port 10096, which is what the configuration file states. To avoid changing the *generic_config* file in the repository, We can copy it to the component's home directory, So changes will remain untouched by future git pulls:

    cp ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    cp /etc/generic_config config

So, to begin we type:

    cd ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    bin/apriltagscomp --Ice.Config=config
    
If the robot's camera is pointing towards one of the tags, You should see in the terminal lines showing the ID and pose of each visible tag.


