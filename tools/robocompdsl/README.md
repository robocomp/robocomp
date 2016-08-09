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

Components communicate through interfaces, so we have to generate an IDSL. Let's start with a simple but complete IDSL named *Plane.idsl*:

        import "Point.idsl";
        module RoboCompPlane
        {
          sequence<RoboCompPoint::PointXY> Points;
        
          struct Dimensions
          {
            int width;
            int height;
          };
          
          interface planeSetup
          {
        	void setWidth(int width, out Dimensions dims);
        	void setHeight(int height, out Dimensions dims);
        	void setName(string name,  out string lastName);
        	void addPoint(RoboCompPoint::PointXY point,  out Points pts);
          };
            interface Plane
          {
        	void newPlaneName(string planeName);
        	void newDimensions(Dimensions dims);
          };
        };

That IDSL imports other IDSL named *Point.idsl* which only has a structure:

        module RoboCompPoint
        {
        	struct PointXY
        	{
        		int x;
        		int y;
        	};
        };

If we want create an IDSL compatible with ROS and ICE Middlewares, we must pay attention to 3 restrictions.

    1.- Services have two parameters and return void/bool.
    2.- Topics have a single parameter and return void.
    3.- Only simple data structures like primitive types (int, float, etc.), structures or arrays are allowed.

Any IDSL which follow the restrictions, can be used for both ROS as ICE.

## ICE Middleware Components

## ROS Middleware Components

## Future improvements

  1.- Generate .ice files.
  
  2.- generate components which can use ICE/ROS Middleware with same interface.
  
  3.- We will be happy to consider all your suggestions.
