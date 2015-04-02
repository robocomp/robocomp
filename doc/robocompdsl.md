#Using robocompdsl

**robocompdsl** is the new tool used in RoboComp to automatically generate components and modify their main properties once they have been generated (e.g., communication requirements, UI type). It is one of the core tools of the framework so, if you installed RoboComp, you can start using it right away.

This new version can only be used from the command line, but the languages used to define components and their interfaces remain the same: **CDSL** to specify components and **IDSL** to specify interfaces. Take a look to the tutorial ["a brief introduction to Components"](components.md) for an introduction to the concept of component generation and the languages involved.

There are three tasks we can acomplish using **robocompdsl**: generating a CDSL template file, generating the code for a previously existing CDSL file, and regenerating the code for an already generated component.

## Generating a CDSL template file
Even tough writing CDSL files is easy --their structure is simple and the number of reserved words is very limited-- robocompdsl can generate template CDSL files to be used as a guide when writing CDSL files.

 $ robocompdsl path/to/file/mycomponent.cdsl

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



 
## Generating a component given a CDSL file



## Updating a component given a CDSL file







