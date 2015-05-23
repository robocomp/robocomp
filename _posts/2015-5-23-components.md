---
layout: post
title: Software components in RoboComp, A brief introduction
categories: [Post]
tags: [Software Components]
description: Two major problems encountered when creating large, complex software are scalability and reusability. These problems become especially acute when it comes to writing the software that controls today robots. Robotics is the mixed bag of technology, where almost everything finds its way through. Also, Robotics is...
---

#Software components in RoboComp, a brief introduction

Two major problems encountered when creating large, complex software are scalability and reusability. These problems become especially acute when it comes to writing the software that controls today robots. Robotics is the mixed bag of technology, where almost everything finds its way through. Also, Robotics is the place where our dreams of intelligent machines meet, in an endless attempt to build a truly useful tool for our daily lives. Because of this, we organize the software for our robots in big architectures that try to reproduce whatever we understand by intelligent behavior. The most audacious architectures are called *cognitive architectures* and try to integrate all levels of behavior and reasoning needed to achieve intelligence. Some of them have been with us for more than 30 years, [SOAR.](http://soar.eecs.umich.edu/)

The problem with building these little monsters is that you need a very powerful underlying infrastructure that lets you build and modify software created by many people and that has to execute on real, moving machines. Also, everybody expect robots to be smarter than they really are and that is a lot of preassure. Components provide a new, developing technology that can be very helpful here. Components are *programs that communicate* and as such, they are built with everything at hand, libraries, objects, threads, sockets, lambda functions and any other thing you can come up with to code a program. Also, components need a way to communicate among them and here is where communication middlewares get in. If you want to communicate programs written in different languages, running across the internet, executing on different hardware architectures -even browsers- then you cannot get along with a socket. You really need a middleware. Putting together these to elements, programs and a communication middleware, you almost come up with a component. One more thing is needed, a model for your components. You need to define what is a generic component and consequently how its internal structure is going to be, its directory and building ecosystem, how it has to be documented, its default behavior, how it will be deployed and its modes of communicating. There are several proposals that do exactly this, being the most famous [CORBA.](http://www.corba.org)

In RoboComp (2005-) we have created our own component model, inspired by the [ORCA](http://orca-robotics.sourceforge.net/) model and making it evolve to fit our needs along these years. As a middleware, RoboComp primarily uses [Ice](www.zeroc.com) and there is ongoing experimental work to make RoboComp middleware agnostic, so its components can be re-generated to use other middlewares such as [DDS](http://portals.omg.org/dds/).

RoboComp's components model is quite simple and we always try to simplify it even more. It can be best explained through two Domain Specific Languages (DSLs) that have been created to define a component at a very high level of abstraction. **IDSL** stands for "Interface Definition Specific Language" and currently is a subset of Ice's Slice interface language. With IDSL you write the data structures and functions that a component can implement, require, subscribe to or publish. A component can implement several interfaces, offering different views of its internal functioning. Also, the same interface can be implemented by many components. This is an example of a simple interface written in IDSL:

    module RoboCompSpeech
    {
      interface Speech
      {
         bool say(string text,bool overwrite);
         bool isBusy();
      };
    };

**CDSL** stands for "Component Definition Specific Language" and allows the user to specifiy its name, accesible interfaces, communication connections, target language and other available modules or libraries that you want to include in the building scripts.

    import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
    import "/robocomp/interfaces/IDSLs/Laser.idsl";
    Component prueba
    {
        Communications
        {
          requires DifferentialRobot, Laser;
        };
        language cpp;
        gui Qt(QWidget);
    };

Using these two DSLs, RoboComp can generate the source code of the component using a tool designed to this end. The complete, functioning code of a component is created ready to be compiled and executed. We use a smart inheritance mechanism to separate the generic stuff from the user specific stuff and, based on it, the next time you generate a component, your code will remain untouched but access to new defined proxies will be there.


