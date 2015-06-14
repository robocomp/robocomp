---
layout: post
title: aprilTagsComp, Tutorial to simulate virtual apriltags
categories: [Tutorial]
tags: [AprilTags]
permalink: /april
description: If you haven't already, Then do read about aprilTagsComp[here](apriltags.md )for better understanding. In this tutorial you will learn the actual functionality of apriltags...
---


#aprilTagsComp : Tutorial to simulate virtual apriltags

If you haven't already, Then do read about aprilTagsComp [here](apriltags.md ) for better understanding. In this tutorial you will learn the actual functionality of apriltags.

First make sure you have installed apriltags. Please follow the steps that is given in *INSTALL_APRILTAGS_LIB.TXT*. Then move to the apriltagscomp folder

	cd ~/robocomp/components/robocomp-robolab/components/apriltagsComp
    
Compile by executing

	cmake.
	make

Now that you have compiled the component and have the binary generated. Open a new tab in yakuake or terminal and execute

	cd robocomp/files/innermodel
	rcis simpleworld.xml

Here we will considering *simpleworld.xml* as an example since it has virtual apriltags and a robot with a camera is present for simulation. After execution you should now see two windows, One showing the robot camera's view pointing at one of the apriltags and the other with the site map showing the robot and two virtual apriltags.

Now go back to the terminal where you had compiled the apriltagsComp and execute

	bin/apriltagscomp --Ice.Config=etc/generic_config

You should now see the following output.

```
user@username:~/robocomp/components/robocomp-robolab/components/apriltagsComp$ bin/apriltagscomp --Ice.Config=etc/generic_config
[/home/username/robocomp/classes/rapplication/rapplication.cpp]: Loading [camera:tcp -h localhost -p 10001] proxy at 'CameraProxy'...
18:20:19:421::Info::apriltagscomp.cpp::139::/home/username/robocomp/components/robocomp-robolab/components/apriltagsComp/src/apriltagscomp.cpp::run::CameraProxy initialized Ok!
[/home/username/robocomp/classes/rapplication/rapplication.cpp]: Loading [rgbd:tcp -h localhost -p 10096] proxy at 'RGBDProxy'...
18:20:19:421::Info::apriltagscomp.cpp::150::/home/username/robocomp/components/robocomp-robolab/components/apriltagsComp/src/apriltagscomp.cpp::run::RGBDProxy initialized Ok!
[/home/username/robocomp/classes/rapplication/rapplication.cpp]: Loading [rgbdbus:tcp -h localhost -p 10239] proxy at 'RGBDBusProxy'...
18:20:19:421::Info::apriltagscomp.cpp::161::/home/username/robocomp/components/robocomp-robolab/components/apriltagsComp/src/apriltagscomp.cpp::run::RGBDBusProxy initialized Ok!
18:20:19:423::Debug::genericworker.cpp::53::/home/username/robocomp/components/robocomp-robolab/components/apriltagsComp/src/genericworker.cpp::setPeriod::Period changed100
18:20:19:423::Info::specificmonitor.cpp::56::/home/username/robocomp/components/robocomp-robolab/components/apriltagsComp/src/specificmonitor.cpp::initialize::Starting monitor ...
InputInterface RGBD
AprilTagsFamily tagCodes36h11
ID:0-10 0.17
ID:11-20 0.17
ID:21-30 0.17
InnerModelPath /home/robocomp/robocomp/files/innermodel/simpleworld.xml
RoboCompAprilTagsComp::AprilTagsComp started
InnerModelReader: reading /home/robocomp/robocomp/files/innermodel/simpleworld.xml
InnerModelRGBD: 0.000000 {10096}
"/home/robocomp/robocomp/files/innermodel/simpleworld.xml"   "rgbd" 
FOCAL LENGHT: 480 
  6.45862 fps
  7.17213 fps
  7.09036 fps
  6.94859 fps
  7.09561 fps
  7.01433 fps
  7.09569 fps
  7.02332 fps
  7.24122 fps
  7.17631 fps
  6.85862 fps
  7.00415 fps
  7.14883 fps
  6.92038 fps
  7.03233 fps
```



