
RoboCompDSL's User Guide
============

# Table of Contents
1. [Introduction](#introduction)
2. [Generating a CDSL template file](#generating-a-cdsl-template-file)
	1. [Parameters description](#1--parameters)
	2. [The SMDSL (State Machine DSL) file](#2--the-smdsl-state-machine-dsl-file)
	3. [Regenerating a component](#3--regenerating-a-component)
3. [Generating an IDSL file](#generating-an-idsl-file)
    1. [Restrictions](#restrictions)
    2. [Basic types for ICE & ROS](#basic-types-for-ice--ros)
4. [Generating an ICE file from an IDSL](#generating-an-ice-file-from-an-idsl)
5. [ICE Middleware Components](#ice-middleware-components)
    1. [1.- Require / Implement](#require--implement)
    2. [2.- Publish / Subscribe](#publish--subscribe)
6. [ROS Middleware Components](#ros-middleware-components)
    1. [1.- Require / Implement](#require--implement-1)
    2. [2.- Publish / Subscribe](#publish--subscribe-1)
7. [ICE/ROS Middleware Interfaces](#iceros-middleware-interfaces)

## Introduction

**RoboCompDSL** is a tool used in RoboComp to automatically generate interfaces and components and modify their main properties once they have been generated (e.g., communication requirements, UI type).
It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away.
The languages used to define components and their interfaces remain mostly the same: **CDSL** to specify components and **IDSL** to specify interfaces.
If you still don't know what a component or interface is in Robocomp you should probably review the [tutorials](https://github.com/robocomp/robocomp/blob/stable/doc/README.md).
This one is specific for [robocompdsl](https://github.com/robocomp/robocomp/blob/stable/doc/robocompdsl.md).

<small><i>Note: If your a looking for Development documentation you can find it <a href='./DEVELOPER.md'>here</a>.</i></small>


## Generating a CDSL template file

RoboCompDSL can generate template CDSL files to be used as a guide when writing CDSL files.

    $ robocompdsl path/to/mycomponent/mycomponent.cdsl

This will generate a CDSL file with the following content:

    import "import1.idsl";
    import "import2.idsl";

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

> Just copy **cdsl.xml** to /home/**your-linux-user**/.kde/share/katepart/syntax/cdsl.xml (KDevelop)

> and restart your application.

### 1.- Parameters 
* Communications InterfaceName : Create communications
	* implements: Create RPC server 
	* requires: Create proxy to RPC server
	* publishes: Create publication proxy a specific topic
	* subscribesTo: Subscribe a specific topic
* Language
	* CPP: Implement component using C++
	* CPP11: Implement component using C++ 11 to allow use new Ice implementation  (https://doc.zeroc.com/ice/3.7/language-mappings/c++11-mapping)
	* Python: Implement component using python 2.7 
* Gui (WindowType): Include visual interface using different window types
	* QWidget: https://doc.qt.io/qt-5/qwidget.html 
	* QDialog: https://doc.qt.io/qt-5/qdialog.html
	* QMainWindow: https://doc.qt.io/qt-5/qmainwindow.html
* Optionals: 
	* agmagent: Include Cortex-Agent communication patterns
	* InnerModelViewer: Include innermodelViewer resources (https://github.com/robocomp/robocomp/tree/stable/libs/innermodel)
	* statemachine: It must be followed by a path to a valid smdsl file that would describe the state machine that will be implemented on the component.
	
### 2.- The SMDSL (State Machine DSL) file
The smdsl is a domain-specific language that let us define the State Machine that would be in charge of the main functionality of the component behaviour.  
If you want to know more about the State Machine Framework you can read about this on in the [QT Documentation](https://doc.qt.io/qt-5/statemachine-api.html).  
Currently this language is defined like this:

```
name_machine{
  [states name_state *[, name_state];]
  initial_state name_state;
  [end_state name_state;]
  [transition{
      name_state=>name_state *[, name_state];
      *[name_state=>name_state *[, name_state];]
  };]
};

*[ :father_state [parallel]{
  [states name_state *[, name_state];]
  [initial_state name_state;]
  [end_state name_state;]
  [transition{
      name_state=>name_state *[, name_state];
      *[name_state=>name_state *[, name_state];]
  };]
};]


```
`[]`    → optionality  
`*`   → Item List

A complete example of the content of a valid smdsl file would be like this:

```
Machine_testcpp{
    states test2, test3, test4, test5;
    initial_state test1;
    end_state test6;
    transitions{
	test1 => test1, test2;
	test2 => test3, test5, test6;
	test3 => test3, test4;
	test4 => test5;
	test5 => test6;
    };
};

:test1 parallel{
    states test1sub1, test1sub2;
    transitions{
	test1sub1 => test1sub1;
	test1sub2 => test1sub2;
    };
};

:test1sub2{
    initial_state test1sub21;
    end_state test1sub22;
    transition{
	test1sub21 => test1sub21,test1sub22;
    };
};

:test3 parallel{
    states test3sub1, test3sub2, test3sub3;
    transitions{
	test3sub1 => test3sub1;
	test3sub2 => test3sub2;
    };
};

:test5{
    states test5sub2;
    initial_state test5sub1;
    transitions{
	test5sub1 => test1sub2;
	test1sub2 => test5sub1;
    };
};

```

If the CDSL file have the `statemachine "filename.smdsl"` option in it and point to a valid smdsl the robocompdsl tool will create the defined states
and transitions defined on this smdsl file.

When you create a component with the State Machine definition some new methods and attributes are added to your specificworker class.  
You will see methods with the `sm_` prefix created for each state defined in the smdsl file.  
You will also have available a signal for each possible transition of your state machine.  
Those signals are attributes of your specificworker class and have the `t_` prefix and the name have a syntax like this: `t_state1_to_state2`.  
If you want to transition from a state to another you need to call the emit of that signal with something similar to this:
```python
# python
self.t_state1_to_state2.emit()
```
or
```c++
// c++
this.t_state1_to_state2.emit();
```

In the special case of using the DefaultMachine generated with the initial smdsl file, some emit are automatically added for you to call init and compute state on the way it was done previous to the State Machine framework.

> Note: Most of the implementation of the State Machine behaviour was a contribution from [@ibarbech](https://github.com/ibarbech) during his participation on [GSoC 2016](https://robocomp.github.io/web/gsoc/2016/index).    



### 3.- Regenerating a component
If you need to make any change in the cdsl file to include new interfaces or options or if you make changes to the smdsl file you will need to regenerate the component to get those changes on the code of your component.
the component to get those changes on the code of your component.  
`robocompdsl` will only overwrite the main file and the generic* versions of the other files. The specific* version of the files will not be overwrited
and a *.new file will be created side to side to your modified files.  
`robocompdsl` currently also have an execution option `-d` where a diff command can be provided and used to show a comparation of the old and new file versions and the the changes can be incorporated easily.
This way if execute
```bash
robocompdsl -d meld component.cdsl . 
```  
the meld diff tool ui will be launched for each existing specific file and its *.new version.  
Tools like meld gives the user the option copy lines from a file to another and the save the changes. 
  

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


### Restrictions

If we want to create an IDSL compatible with ROS and ICE Middlewares, we must pay attention to 3 restrictions.

    1.- Services have two parameters and return void/bool.
    2.- Topics have a single parameter and return void.
    3.- Only simple data structures like primitive types (int, float, etc.), structures 
    or arrays are allowed.

Any IDSL which follow the restrictions, can be used for both ROS as ICE.

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


## Generating an ICE file from an IDSL

If we are going to use ICE Middleware, we need create our .ice files like we did with .idsl files. Let's use RoboCompDSL for this:

    $ robocompdsl path/to/myIDSL/myIDSL.idsl output/path/iceFile
    
**Remember to save your .ice file in (~/robocomp/interfaces/).**


## ICE Middleware Components

In this section we will create components which communicate via ICE Middleware using *Plane.idsl*.


### 1.- Require / Implement

We will use these CDSL:

    import "Plane.idsl";
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

    import "Plane.idsl";
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


### 2.- Publish / Subscribe

As we did above, we are going to create our CDSLs for this case:

    import "Plane.idsl";
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

    import "Plane.idsl";
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

    $ robocompdsl path/to/mycomponent/mycomponent.cdsl output/path
    
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

## ROS Middleware Components

In this section we will create components which communicate via ROS Middleware using *Plane.idsl*.
The use of the types is a little different to what we saw with ICE Middleware.


### 1.- Require / Implement

Our CDSLs:

    import "Plane.idsl";
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

    import "Plane.idsl";
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


### 2.- Publish / Subscribe

Publshers/Subscribers have less difficulty. Let's create our CDSLs like ICE but with **(ros)**:

    import "Plane.idsl";
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

    import "Plane.idsl";
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

## ICE/ROS Middleware Interfaces

If we want to use an interface through Ros and Ice Middlewares (for example: *requires Plane(ice), Plane(ros);* ), we need to pay attention to a couple of changes.

Our proxys will be called (only if we have an interface through ROS and ICE Middleware):
interface_proxy      (ICE like we used to)
interface_**ros**proxy   (ROS)

Because of this improvement, the ROS data types and methods **always** be called ended with ROS:

RoboCompPlane**ROS**::Dimensions dims;
void SpecificWorker::newDimensions**ROS**(RoboCompPlane**ROS**::Dimensions dims)
    
The call to this methods with our proxys are like this:

    interface_proxy->newDimensions(dims)        (ICE)
    interface_rosproxy->->newDimensions(dims)   (ROS)
    
**ROS word doesn't appear in the call.**
