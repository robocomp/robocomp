RoboCompDSL's User Guide
============

## Introduction

**RoboCompDSL** is a tool used in RoboComp to automatically generate components and modify their main properties once they have been generated (e.g., communication requirements, UI type). It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away. The languages used to define components and their interfaces remain mostly the same: **CDSL** to specify components and **IDSL** to specify interfaces

## Generating a CDSL template file

RoboCompDSL can generate template CDSL files to be used as a guide when writing CDSL files.

    $ robocompdsl path/to/mycomponent/mycomponent.cdsl

This will generate a CDSL file with the following content:

    import "/robocomp/interfaces/IDSLs/import1.idsl";
    import "/robocomp/interfaces/IDSLs/import2.idsl";

    Component CHANGETHECOMPONENTNAME
    {
        Communications
        {
            implements interfaceName;
            requires otherName;
            subscribesTo topicToSubscribeTo;
            publishes topicToPublish;
        };
        language Cpp;
        gui Qt(QWidget);
    };

## Generating an IDSL file

## ICE Middleware Components

## ROS Middleware Components

## Future improvements

  1.- Generate .ice files.
  
  2.- generate components which can use ICE/ROS Middleware with same interface.
  
  3.- We will be happy to consider all your suggestions.
