# CDSL
CDSL is Component Description Specific Language and is used to create and maintain their component descriptions from a textual model. CDSL files contain the basic definition of the structure of the component and information about communication parameters such as proxies, the programming language of the component, interfaces and topics used by the components, the optional support of Qt graphical interfaces.

## Structure of a CDSL
CDSL permits comments in C and C++ style, starting with //

Every CDSL file contains the following properties:

#### Interfaces and data types defined in external IDSL files.
```
import "import1.idsl";
import "import2.idsl";
```
#### Component name.
```
Component myComponent
```
#### Communication model: 
- Required and provided interfaces.
- Topics that the component will publish or subscribe to.
```
Communications
        {
                implements interfaceName;
                requires otherName;
                subscribesTo topicToSubscribeTo;
                publishes topicToPublish;
        }
```

#### Programming language of the component.
```
language Cpp//Cpp11//Python;
```
#### Graphical interface support.
```
gui Qt(QWidget//QDialog//QMainWindow);
```
#### Dependences with external classes and libraries.
```
options agmagent;
options InnerModelViewer;
```
## 

