# Frequently Asked Questions

## What is Robocomp?

Robocomp is an Open-Source Robotics Framework, where you can simulate, model and program your own robots, in a C.O.P (Component Oriented Programming) enviroment. You can create components and share them with the community. Components communicate with each other via Ice [[1](https://doc.zeroc.com/ice/3.7/introduction)], a framework for building distributed applications. Simulation is done with RCIS, the built-in simulator.

Robocomp was presented with the publication of the paper: "Robocomp: A Tool-Based Robotics Framework" [[2](https://www.researchgate.net/publication/262179907_RoboComp_A_ToolBased_Robotics_Framework_Lecture_Notes_in_Computer_Science)]. You can see more projects from Robolab, the investigation team from Universidad de Extremadura that created and currently maintains Robocomp [here](https://robolab.unex.es) (some of the information is in Spanish).

## What to do if you are new to the project and want to help?

Thank you for wanting to improve the project! 

So you are informed, please first read [the Contributing Guide](../CONTRIBUTING.md), and the documentation for the project. [Here](https://robocomp.github.io/web/blog) you can see the list of projects for Google Summer of Code for some ideas on what is needed. If you need additional help, drop a message on [Google Groups](https://groups.google.com/forum/?hl=es#!forum/robocomp-dev) or [Gitter](https://gitter.im/robocomp).

If you make your own project with Robocomp, we would love to hear about it. Tell us on [Gitter](https://gitter.im/robocomp).

## What kind of work can you do to improve Robocomp?

You can contribute with all types of skills. Web design for the webpage, technical documentation for the docs, UX and UI design for the simulator, image processing to make the Learnbot more interactive for kids, programming to make Robocomp work with other languages like JavaScript, and many more. You can help in any way you want and are able to.

See the projects done in [Google Summer of Code](https://robocomp.github.io/web/blog/) for some ideas.

## In which branch should you make changes?

Our up-to-date branch is currently `development`. If you are going to file an issue or make an improvement, this is the branch to do it. 

## What is the Learnbot?

[`Taken from the wiki:`](https://github.com/robocomp/learnbot/wiki) LearnBot is a social low cost robot that has been designed in to the area of educational robotics for promoting the development of computational thinking in diferent educational stages, specically through the learning of the Python language. It has being entirely built using a robotics framework developed by our Robotic Laboratory.

But Learnbot is part of a more ambitious ecosystem, Learnblock, which is being created as a facilitator environment to encourage current teacher swith out programming knowledge to learn the basics of programming, it is an IDE similar to Scrach, at least into a level where they can introduce their students into the digital world.

## How should I download Robocomp?

Follow the install guide from the [Readme page](https://github.com/robocomp/robocomp/blob/development/README.md), or for a faster way, use [this installation script](https://github.com/robocomp/robocomp/blob/development/doc/installScript.md). 

To use the installation script you need to download the Raw of the file, then from the terminal:

```bash
# To check that the script is there
ls
# Make it executable
chmod +x installScript.sh
# Then you can run it with
./installScript.sh
```

For a more detailed explanation, please see the [tutorial](https://github.com/robocomp/robocomp/blob/development/installScript.sh).

## How can Windows, Arch Linux, Linux Mint or any other Operating System that is not Ubuntu be used?

At the moment is not possible to use Robocomp directly in another Linux distro or on Windows. However, you can use virtualization and use Ubuntu inside your machine!

If your computer have at least 8 GB of RAM, you can use VMware or VirtualBox to have an Ubuntu image running inside your OS.

You need to download an Ubuntu image from [OS boxes](https://www.osboxes.org/ubuntu/), run either VMware or VirtualBox, and load it. After that you will have a perfect Ubuntu system running, from where you can use Robocomp.

## How can you create your own 3D Models?

These are written in *Extensible Markup Language* [XML](https://www.ibm.com/developerworks/library/x-newxml/index.html), a standard markup language which help you create your own elements.

First you need to load your 3D models. There are a few pages where you can download them, like 3DSkys or Google's 3D Warehouse. 

With XML you set the name, size and place of the 3D objects inside the simulation, with IDs for them. Then you can place items with the `mesh id`, the .3ds file you uploaded previously.

An example of placing a table inside a room would be:

```XML
<innermodel>
<transform id='room'>
  <include path="/home/robocomp/models/RoboKid"/>
<transform id="table" tx="3800" tz="3200" ty="800">
		<mesh id="mesaR" file="/home/robocomp/robocomp/files/osgModels/basics/cylinder.3ds" scale="600,600,12" rx="1.5707" collide="1" />
		<mesh id="mesaRb" file="/home/robocomp/robocomp/files/osgModels/basics/cylinder.3ds" scale="30,30,400" rx="1.5707" ty="-400" collide="1" />
		<mesh id="mesaRbb" file="/home/robocomp/robocomp/files/osgModels/basics/cylinder.3ds" scale="325,325,9" rx="1.5707" ty="-800" collide="1" />
		<plane id="mesaTag" ny="1" py="11" size="70,-70,5.0" texture="/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/autonomyLab/30.png" />
		<plane id="mesaTagb" ny="1" py="11" size="80,-80,3.5" texture="#ffffff" />
	</transform>
</innermodel>  
```

With a few basic geometric shapes it's possible to create a lot of more complicated models. 

## How should you file an Issue?

Please see our ['Contributing'](../CONTRIBUTING.md) file for more information.

### If you don't see an answer to your question here, please file an Issue or tell us at robocomp.team@gmail.com.



