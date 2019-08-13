# Robocompdsl-GUI User Guide

Introduction
---
Robocompdsl-GUI is a graphical user interface developed for Robocompdsl, a tool used in RoboComp to automatically generate interfaces and components and modify their main properties once they have been generated. 

The aim of this graphical interface is to make easier the task of creating and modifying a component. The language used to define components is CDSL, so the main part of the interface is a text editor, where the code to create the component is written.

CDSL structure
---
The CDSL file structure will be explained in the following paragraphs. To better understand its layout, it is attached an example file below:

```
import "Laser.idsl";
import "DifferentialRobot.idsl";

component my_component
{
	communications
	{
		implements DifferentialRobot;
		publishes Laser;
	};
	language cpp;
	gui Qt(QWidget);
};
```

This file is organized in different blocks:
- Imports: idsl files loaded to use the interfaces specified in “Communications”.
- Component name: Name of the component.
- Communications: Create communications.
    - implements: Create RPC server.
    - requires: Create a proxy to RPC server.
    - publishes: Create publication proxy a specific topic.
    - subscribesTo: Subscribe a specific topic.
- Language
    - CPP: Implement component using C++
    - CPP11: Implement component using C++ 11 to allow use new Ice implementation (https://doc.zeroc.com/ice/3.7/language-mappings/c++11-mapping)
    - Python: Implement component using python 2.7
- Optionals:
    - Gui (WindowType): Include visual interface using different window types.
       - QWidget: https://doc.qt.io/qt-5/qwidget.html
       - QDialog: https://doc.qt.io/qt-5/qdialog.html
       - QMainWindow: https://doc.qt.io/qt-5/qmainwindow.html
    - agmagent: Include Cortex-Agent communication patterns.
    - InnerModelViewer: Include innermodelViewer resources. (https://github.com/robocomp/robocomp/tree/stable/libs/innermodel)

Graphical Interface Elements and Functions
---

Robocompdsl-gui layout is organized as we see in the following picture:


The graphical interface elements are grouped into different parts: top, left side, right side and bottom of the window.

### Top of the window:
- Component name: a line text edit to enter the component name.
- Load cdsl: this button opens a file browser to select and load an existing cdsl file into the main editor.
- Select directory: the path of the directory to save the component. It can be specified by writing the path in the line text editor or pushing the button “Select Directory” and selecting the directory in a file browser.

### Left side of the window:
- Communication type combo box: to select the communication type. The options are:
    - publishes
    - implements
    - subscribesTo
    - requires
- Interfaces list widget: to select the interface with the selected communication type.
    - Clicking with the right mouse button: add one element. When an interface is added using the list widget, the necessary imports are loaded automatically.
    - Clicking with the left mouse button: remove one element. When an interface is removed using the list widget, the necessary imports are removed automatically.
    - Pushing Ctrl and clicking on different interfaces will add/remove an interface to a group of them with the selected communication type.

### Central part of the window:
- Main text editor: to write the code of the cdsl file manually.
    - Note: In the current version of Robocompdsl-gui it is not possible to change “options” manually.

### Right side of the window:
- Language combo box: to select the language in which the component will be created:
    - Python
    - Cpp
    - Cpp11
- Optional parameters:
    - GUI: add visual interfaces using window types. When the checkbox is checked the combo box turns to enable to choose a GUI option:
        - QWidget
        - QDialog
        - QMainWindow
    - Agmagent checkbox: add/remove admagent configurations.
    - InnerModelViewer checkbox: add/remove innerModelViewer configurations.

### Bottom of the window:
- Console: it displays error and feedback messages. When the component is correct a message is printed on the console and the component can be created safely.
- Reset button: to load an empty template in the main text editor.
- Create button: to create a cdsl file with the content of the editor.
- Generate Component button: to generate a component using the tool robocompdsl.

How to create a component using Robocompdsl-GUI
---
The steps to create a component correctly are:
- Enter the name in the text line editor.
- Select a directory entering it in the line editor or pushing the “Select directory” button and selecting the folder.
- Select a communication type from interface Combo box.
- Select an interface from the list widget to add with the selected communication type. The imports will be added automatically
- Select language using the combo box
- Add optional parameters if needed
- Use the “Reset” button to load an empty template
- Use the “Create” button to create a cdsl file with the content of the editor
- Use the “Generate Component” button to generate a component using the tool robocompdsl

How to write a cdsl file manually
----

One of the most interesting features of robocompdsl-gui is that it allows to edit the current cdsl file using the interface or manually. If the user wants to modify the text manually, the interface will display the keywords and errors in different colors using a text highlighter.

Robocompdsl-gui uses text blocks to identify if the grammar is written correctly. To be sure you don’t make a mistake please follow these grammatical guides:

```
import "interface_file_name.idsl";
import "interface2_file_name.idsl";

component component_name
{
	communications
	{
		communication_type interface1, interface2;
		communication_type2 interface2;
	};
	language python/cpp/cpp11;
	gui Qt(QWidget/QDialog/QMainWindow);
	options agmagent, innermodelviewer;
};
```

This feature is still in beta version, so there are some restrictions when modifying the editor manually, that are listed below:
“options”: It is not possible to change this block manually in the current version.

Extra features
---
- Syntax Highlighter: when the user writes a corret keyword, it will be hilighted in blue bold font.   
- Error Highlighter: when the user write s wrong word he will be warn with an red underlined highlight.
- Console: information message, wrong configuration warnings and errors will be displayed on the console.
- Automatic imports: every time that a user selects an interface from the list widget, the respectives idsl file will be imported automatically.
- Closing the application from command line: pressing Ctrl-C from the command line will close the application.

Restrictions
---
Rbocompdsl-GUI was developed using the following language and libraries versions:
- Python 3.6
- PyParsing 2.2
- PySide2 (v. 5.12)

>For further information about Robocompdsl please read the Robocompdsl User Guide.
