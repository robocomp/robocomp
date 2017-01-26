RoboCompDSL's User Guide
============

# Table of Contents
1. [Introduction](#introduction)
2. [Generating a CDSL template file](#cdsl)
3. [Generating an IDSL file](#idsl)
    1. [Restrictions](#restrictions)
    2. [Basic types for ICE & ROS](#btypes)
4. [Generating an ICE file from an IDSL](#iceFile)
5. [ICE Middleware Components](#ice)
    1. [1.- Require / Implement](#Ireq-imp)
    2. [2.- Publish / Subscribe](#Ipub-sub)
6. [ROS Middleware Components](#ros)
    1. [1.- Require / Implement](#Rreq-imp)
    2. [2.- Publish / Subscribe](#Rpub-sub)
7. [ICE/ROS Middleware Interfaces](#ice/ros)

<div id='introduction'/>
## Introduction

**RoboCompDSL** is a tool used in RoboComp to automatically generate components and modify their main properties once they have been generated (e.g., communication requirements, UI type). It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away. The languages used to define components and their interfaces remain mostly the same: **CDSL** to specify components and **IDSL** to specify interfaces

<div id='cdsl'/>
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

> If you are using [Kate](https://kate-editor.org) or [KDevelop](https://www.kdevelop.org), you can add our [Syntax Highlighting File](https://github.com/robocomp/robocomp/blob/highlyunstable/tools/robocompdsl/kate/cdsl.xml).

> Just copy **cdsl.xml** to /home/**your-linux-user**/.local/share/katepart5/syntax/cdsl.xml (Kate)

> Just copy **cdsl.xml** to /home/**your-linux-user**/share/apps/katepart/syntax/cdsl.xml (KDevelop)

> and restart your application.

<div id='idsl'/>
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

<div id='restrictions'/>
### Restrictions

If we want create an IDSL compatible with ROS and ICE Middlewares, we must pay attention to 3 restrictions.

    1.- Services have two parameters and return void/bool.
    2.- Topics have a single parameter and return void.
    3.- Only simple data structures like primitive types (int, float, etc.), structures 
    or arrays are allowed.

Any IDSL which follow the restrictions, can be used for both ROS as ICE.

<div id='btypes'/>
### Basic types for ICE & ROS

We know that ROS uses types like int8, int16, float16, etc. The following table shows a comparison of DSL types for ICE and ROS:

| IDSL Types    | ICE Types     |  ROS Types    |
| :-----------: | :-----------: | :-----------: |
| bool          | bool          | std_msgs/Bool |
| byte          | byte          | std_msgs/Byte |
| short         | short         |      ---      |
| int8          |      ---      | std_msgs/Int8 |
| int16         |      ---      | std_msgs/Int16|
| int / int32   | int           | std_msgs/Int32|
| int64         |      ---      | std_msgs/Int64|
| long          | long          |      ---      |
| float8        |      ---      |std_msgs/Float8|
| float16       |      ---      |std_msgs/Float16|
| float / float32| float         |std_msgs/Float32|
| float64       |      ---      |std_msgs/Float64|
| double        | double        |      ---      |
| string        | string        |std_msgs/String|
| time          |      ---      |std_msgs/Time  |
| empty         |      ---      |std_msgs/Empty |

<div id='iceFile'/>
## Generating an ICE file from an IDSL

If we are going to use ICE Middleware, we need create our .ice files like we did with .idsl files. Let's use RoboCompDSL for this:

    $ robocompdsl path/to/myIDSL/myIDSL.idsl output/path/iceFile
    
**Remember save your .ice file in (/robocomp/interfaces/).**

<div id='ice'/>
## ICE Middleware Components

In this section we will create components which communicate via ICE Middleware using *Plane.idsl*.

<div id='Ireq-imp'/>  
### 1.- Require / Implement

We will use these CDSL:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeClient
    {
    	Communications
    	{
    		requires planeSetup(ice);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };
    
And implement:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeServer
    {
    	Communications
    	{
    		implements planeSetup(ice);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };
    
You can simply write *requires Plane;* ICE is the default middleware.
Now, let's use RoboCompDSL to generate our components.

    $ robocompdsl path/to/mycomponent/mycomponent.cdsl output/path
    
In your Require component, use this example of *SpecificWorker::compute()*

    void SpecificWorker::compute()
    {
    	Dimensions dims;
    	int width;
    	string name, lastName;
    	width = 300;
    	name = "name1";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    	width = 100;
    	name = "name2";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    	width = 200;
    	name = "name3";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    }

Rewrite all necessary methods in your *specificworker.cpp* of Implement Component. Here is an example:

    void SpecificWorker::setWidth(const int width, Dimensions &dims)
    {
    	printf("changing width value %d\n", width);
    	dims.width = width; //return value (not necessary)
    }
    void SpecificWorker::setName(const string name, string &lastName)
    {
    	printf("changing name value %s\n", name.c_str());
    	lastName = name; //return value (not necessary)
    }
    
Finally, you have 2 components communicating by ICE Middleware. Enjoy!

<div id='Ipub-sub'/>
### 2.- Publish / Subscribe

As we did above, we are going to create our CDSLs for this case:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planePublisher
    {
    	Communications
    	{
    		publishes Plane(ice);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };
    
And subscriber:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeSubscriber
    {
    	Communications
    	{
    		subscribesTo Plane(ice);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };


The process for generating components is similar.

    $ robocompdsl robocompdsl path/to/mycomponent/mycomponent.cdsl output/path
    
You can use this example of *SpecificWorker::compute()* in your Publish Component:

    void SpecificWorker::compute()
    {
    	Dimensions dims;
    	dims.width = 200;
    	dims.height = 300;
    	string name = "newName1";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    	dims.width = 400;
    	dims.height = 400;
    	name = "newName2";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    	dims.height = 600;
    	name = "newName3";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    }

Rewrite all necessary methods in your *specificworker.cpp* of Subscriber Component. Here is an example:

    void SpecificWorker::newDimensions(const Dimensions &dims)
    {
    	printf("New Dimension. Width: %d - Height: %d\n",dims.width, dims.height);
    }
    void SpecificWorker::newPlaneName(const string &planeName)
    {
    	printf("New name. Name: %s\n", planeName.c_str());
    }

As you can see, this section is as above but with minor differences.

<div id='ros'/>
## ROS Middleware Components

In this section we will create components which communicate via ROS Middleware using *Plane.idsl*.
The use of the types is a little different to what we saw with ICE Middleware.

<div id='Rreq-imp'/>  
### 1.- Require / Implement

Our CDSLs:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeClient
    {
    	Communications
    	{
    		requires planeSetup(ros);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };

And Implement:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeServer
    {
    	Communications
    	{
    		implements planeSetup(ros);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };

If you want use ROS Middleware, you must use **(ros)** option for each interface.
After generating component, rewrite our specific code.

An example of *SpecificWorker::compute()* for your Require Component:

    void SpecificWorker::compute()
    {
    	RoboCompPlaneROS::Dimensions dims;
    	std_msgs::Int32 width;
    	std_msgs::String name, lastName;
    	width.data = 300;
    	name.data = "name1";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    	width.data = 100;
    	name.data = "name2";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    	width.data = 200;
    	name.data = "name3";
    	planesetup_proxy->setWidth(width, dims);
    	planesetup_proxy->setName(name, lastName);
    	ros::spinOnce();    //Don't remove this line
    }
    
Notice that ROS uses its own data types and how they are used. It is simple, right?

Rewrite all methods in your Implement Component. An example:

    bool SpecificWorker::setWidthROS(RoboCompPlaneROS::setWidth::Request &req, RoboCompPlaneROS::setWidth::Response &res)
    {
    	printf("Changing width value %d\n", req.width);
    	res.dims.width = req.width; //return value (not necessary)
    	return true;
    }
    bool SpecificWorker::setNameROS(RoboCompPlaneROS::setName::Request &req, RoboCompPlaneROS::setName::Response &res)
    {
    	printf("Changing name value %s\n", req.name.c_str());
    	res.lastName = req.name; //return value (not necessary)
    	return true;
    }
    
2 points (If you are a familiar user with ROS, you will have no problem understanding this):
    1.- ROS Services will always return a bool.
    2.- Your method has 2 parameters (Request and Response).

<div id='Rpub-sub'/>
### 2.- Publish / Subscribe

Publshers/Subscribers have less difficulty. Let's create our CDSLs like ICE but with **(ros)**:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planePublisher
    {
    	Communications
    	{
    		publishes Plane(ros);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };
    
And Subscriber:

    import "/robocomp/interfaces/IDSLs/Plane.idsl";
    Component planeSubscriber
    {
    	Communications
    	{
    		subscribesTo Plane(ros);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };
    
If you want to generate components in Python, change **language Cpp** for **language python**.
An example of *SpecificWorker::compute()* for your Publisher Component:

    void SpecificWorker::compute()
    {
    	RoboCompPlaneROS::Dimensions dims;
    	dims.width = 200;
    	dims.height = 300;
    	std_msgs::String name;
    	name.data  = "newName1";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    	dims.width = 400;
    	dims.height = 400;
    	name.data  = "newName2";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    	dims.height = 600;
    	name.data  = "newName3";
    	plane_proxy->newPlaneName(name);
    	plane_proxy->newDimensions(dims);
    	ros::spinOnce();    //Don't remove this line
    }

Same code that your Require Component.
Example with methods *specificworker.cpp* of Subscriber Component:

    void SpecificWorker::newDimensionsROS(RoboCompPlaneROS::Dimensions dims)
    {
    	printf("new Dimension. Width: %d - Height: %d\n",dims.width, dims.height);
    }
    void SpecificWorker::newPlaneNameROS(std_msgs::String planeName)
    {
    	printf("new Name. Name: %s\n", planeName.data.c_str());
    }

With Subscriber we has no problems with parameters. The use of data types of your parameters is the same as we do in the compute(). So we have minor differences between ICE and ROS Middleware. If you pay attention to the differences shown in this guide, you will not have problems.


Send us any questions or problems you may have.

Check out this tutorial [RoboComp Component & Ros Node chatter](https://github.com/robocomp/robocomp/blob/highlyunstable/doc/RobocompRosChatter.md).

<div id='ice/ros'/>
## ICE/ROS Middleware Interfaces

If we want to use an interface through Ros and Ice Middlewares (for example: *requires Plane(ice), Plane(ros);* ), we need to pay attention to a couple of changes.

Our proxys will be called (only if we have an interface through ROS and ICE Middleware):
interface_proxy      (ICE like we used to)
interface_**ros**proxy   (ROS)

Because of this improvement, the ROS data types and methods **always** be called ended with ROS:

RoboCompPlane**ROS**::Dimensions dims;
void SpecificWorker::newDimensions**ROS**(RoboCompPlane**ROS**::Dimensions dims)
    
The call to this methods with our proxys are like this:

    interface_proxy->newDimensions(dims)            (ICE)
    interface_rosproxy->->newDimensions(dims)   (ROS)
    
**ROS word doesn't appear in the call.**
