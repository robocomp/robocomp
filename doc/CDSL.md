
# CDSL
CDSL is Component Description Specific Language and is used to create and maintain their component descriptions from a textual model. CDSL files contain the basic definition of the structure of the component and information about communication parameters such as proxies, the programming language of the component, interfaces and topics used by the components, the optional support of Qt graphical interfaces.

## Structure of a CDSL
CDSL permits comments in C and C++ style, starting with //

Every CDSL file contains the following properties:

- #### Interfaces and data types defined in external IDSL files.
```
import "import1.idsl";
import "import2.idsl";
```
- #### Component name.
```
Component myComponent
```
- #### Communication model: 
	- Required and provided interfaces.
	- Topics that the component will publish or subscribe to.
   		- implements: Create RPC server.
   		- requires: Create a proxy to RPC server.
		- publishes: Create publication proxy a specific topic.
		- subscribesTo: Subscribe a specific topic.
```
Communications
        {
                implements interfaceName;
                requires otherName;
                subscribesTo topicToSubscribeTo;
                publishes topicToPublish;
        }
```

- #### Programming language of the component.
	- CPP: Implement component using C++
	- CPP11: Implement component using C++ 11 to allow use of [new Ice implementation.](https://doc.zeroc.com/ice/3.7/language-mappings/c++11-mapping)
    - Python: Implement component using python 3.x.
```
language Cpp//Cpp11//Python;
```
- #### Graphical interface support. (Optional)
	- [QWidget](https://doc.qt.io/qt-5/qwidget.html)
	- [QDialog](https://doc.qt.io/qt-5/qdialog.html)
	- [QMainWindow](https://doc.qt.io/qt-5/qmainwindow.html)
```
gui Qt(QWidget//QDialog//QMainWindow);
```
- #### Dependences with external classes and libraries. (Optional)
	- agmagent: Include Cortex-Agent communication patterns.
	- InnerModelViewer: Include [InnerModelViewer](https://github.com/robocomp/robocomp/tree/stable/libs/innermodel) resources.
	- dsr: Include [dsr](https://github.com/robocomp/cortex) resources.
```
options dsr, agmagent, InnerModelViewer;
```
##
