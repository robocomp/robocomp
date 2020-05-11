
RoboCompDSL's Developer Guide
=============================

# Table of Contents
- [RoboCompDSL's Developer Guide](#robocompdsl-s-developer-guide)
- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Main Concepts](#main-concepts)
- [DSLFactory](#dslfactory)
  * [DSL Parsers](#dsl-parsers)
    + [Creating a new parser](#creating-a-new-parser)
    + [PyParsing](#pyparsing)
      - [Pyparsing Parser Creation](#pyparsing-parser-creation)
- [ComponentFacade: AST to Component Object](#componentfacade--ast-to-component-object)
  * [Adding and modifying Component Object attributes](#adding-and-modifying-component-object-attributes)
- [The Template classes](#the-template-classes)
  * [Usage of Python string.Template](#usage-of-python-stringtemplate)
  * [AbstractTemplate class](#abstracttemplate-class)
  * [The *files* and *functions* directories](#the--files--and--functions--directories)
  * [How the Template variables code is generated?](#how-the-template-variables-code-is-generated-)
  * [Creating a new Template](#creating-a-new-template)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>




# Introduction

This is the documentation for developers. It will try to explain some concepts needed to understand robocompdsl code and to be able to modify it or add new features.  

<small><i>Note: If your are looking for documentation about the usage of RobocompDSL as a tool you can find it in the main [README.md](./README.md) file.</i></small>
# Main Concepts
Robocompdsl is a tool to generate component and interface files from dsl files (Domain Specific Languages). 
It is mainly used to generate the necessary files of a component in C++ or Python, from its definition in a .cdsl file.
So we could summarize that the main function of robocompdsl would be this:
```
.cdsl file ==> (SOMETHING) ==> Component (many files)
```

The first step to achieve this goal is to be able to read a .cdsl file that has its own grammar and convert it to an AST (Abstract Syntax Tree) that we can use in python. This is what PyParsing is being used for.   

Pyparsing generates its own structure, PyParsingResult, which can be accessed as an array, dictionary or class as needed.
But it has been decided that for convenience and to control the access to the information of this structure, it should be converted to a specific class of robocompdsl: ComponentFacade. This class can be generated directly from what PyParsing returns and it allows us, through a small hierarchy of classes, to access in a comfortable and ordered way to the information that we will require from the component that we have just read.

In robocompdsl you need parsers for several file types (.cdsl, .idsl, .smdsl). For this reason, 
PyParsing has been encapsulated behind DSLFactory (we will talk about it later) which provides 
the necessary parsers for different file types.

So now we're in this situation:
```
.cdsl file ==> Parser ==> AST ==> ComponentFacade ==> (SOMETHING) ==> Component (many files).
```

Currently, robocompdsl can generate components in c++ and python. The files needed to generate these components are found as Templates and these templates are designed to receive a ComponentFacade type object and from it generate the final files of the component.
So finally we have that the process that is followed internally to pass from a .cdsl to a Component would be this one:
```
.cdsl file ==> Parser ==> AST ==> ComponentFacade ==> Template ==> Component (many files).
```

# DSLFactory
As mentioned above, robocompdsl must be able to read several types of files, each with its own syntax. 
Currently, you can read .cdsl (Component Domain Specific Language), .idsl (Interface Domain Specific Language), 
and .smdsl (State Machine Domain Specific Language) files.
Each of these 3 file types requires a specific parser, but the operations are very similar.
The content of the file is loaded, a parser is applied to it to check the syntax and obtain an 
AST ([Abstract Sintaxt Tree](https://en.wikipedia.org/wiki/Abstract_syntax_tree)).

DSLFactory ([dsl_parsers/dsl_factory.py](./dsl_parsers/dsl_factory.py)) was designed to perform 
these tasks. This class determines which type of file it is processing and calls the appropriate 
parser to finally return the AST corresponding to the dsl file read.
The different parsers can be found in [dsl_parsers/specific_parsers/](./dsl_parsers/specific_parsers/).

In addition to facilitating the reading of all DSL files, DSLFactory was created to work as a 
[Singleton](https://en.wikipedia.org/wiki/Singleton_pattern), so in the whole execution of 
robocompdsl, there can only be one instance of it and it will be shared in all the code points where it is used.
If we add to these a rudimentary cache system that stores the structure that is generated for each parsed file path, 
we have that even if we need several times the content of a dsl file, it will only be parsed 
the first time it is required.

Thus the tasks of DSLFactory are fundamentally these:
* Read the file
* Create the appropriate parser
* Using the parser to generate a structure
* Cache the file path and the generated structure


## DSL Parsers

DSL parsers are in charge of defining the grammar for one of the file types that robocompdsl 
can read and generate the [AST](https://en.wikipedia.org/wiki/Abstract_syntax_tree) from a 
string and that grammar.

To unify and simplify the implementation of new parsers the DSLParserTemplate abstract class was 
created ([dsl_parsers/dsl_parser_abstract.py](./dsl_parsers/dsl_parser_abstract.py)). This class 
implements some of the functionalities common to all parsers and forces the implementation of 
two needed methods.

### Creating a new parser
As mentioned above, robocompdsl offers an abstract class from which you must inherit if you want 
to create a new parser: DSLParserTemplate.
The methods that must be implemented for the parser to work properly are:
```
create_parser(self)
```
In charge of defining and creating the Parser (currently with PyParsing) and:
```
string_to_struct(self, string, **kwargs)
```
In charge of converting the text string, applying the parser, into a convenient [AST](https://en.wikipedia.org/wiki/Abstract_syntax_tree) or structure.

### PyParsing
PyParsing is a python module that allows you to create Parsing Expression Grammars ([PEGs](https://en.wikipedia.org/wiki/Parsing_expression_grammar)) and use them to parse files with that grammar.
You can consult the PyParsing documentation [here](https://pyparsing-docs.readthedocs.io/en/latest/).

Essentially PyParsing offers a series of classes and operators that allow us to generate the grammar and at the end what we have is a parser.

This parser has a parseString method to obtain the AST from the text that is passed to it as a parameter. This AST comes in the form of PyParsingResult nested classes.

#### Pyparsing Parser Creation
If you are going to create your own parser you can check the _create_parser() methods for 
[cdsl](./dsl_parsers/specific_parsers/cdsl/cdsl_parser.py#L17), [idsl](./dsl_parsers/specific_parsers/idsl_parser.py#L15) and [smdsl](./dsl_parsers/specific_parsers/smdsl_parser.py#L14) in these links.

You can also consult other less specific examples of the use of PyParsing in its own [repository](https://github.com/pyparsing/pyparsing/tree/master/examples).

# ComponentFacade: AST to Component Object
In the specific case of the cdsl, given the complexity of the structure generated and 
the need to unify the way to access it, it was decided to create a class from the AST 
returned by PyParsing and that would contain all the necessary information regarding 
that specific component read from the file. 

This class is ComponentFacade ([dsl_parsers/specific_parsers/cdsl/componentfacade.py](./dsl_parsers/specific_parsers/cdsl/componentfacade.py)).

The main purpose of this class is to extend the functionality of the dictionary 
that represents the AST. When this class is instantiated you can pass the AST directly to it
 in the constructor. Each key of that dictionary will become an attribute of the
 ComponentFacade class and they are assigned the value that they contain in the dictionary.
However, some of these keys are mapped so that their value is converted to another 
specific class. This mapping of keys to class types, as well as the subclasses of ComponentFacade itself,
 can be found in the file [dsl_parsers/specific_parsers/cdsl/componentfacade.py](./dsl_parsers/specific_parsers/cdsl/componentfacade.py).

The current representation of this map is this:
```python
CLASS_TYPE_MAP = {
    implements': Interfaces,
    requires': Interfaces,
    subscribesTo': Interfaces,
    'publishes': Interfaces,
    'iceInterfaces': Interfaces,
    rosInterfaces': Interfaces,
    'options': Options,
    'gui': Gui
}
```

The reason for converting both the AST and these values into instances of specific classes is that this allows us to easily define new methods or properties for those classes. These methods can be used as shortcuts to other elements of the component such as:
```python
    def is_agm1_agent(self):
        return self.options.agmagent
```

This solution was also thought to be able to adapt this representation of a Component even if the way it is represented internally is changed, thus maintaining the compatibility of the code.

## Adding and modifying Component Object attributes
If you add new elements to the cdsl grammar or change the expected values of the existing ones this should be reflected in ComponentFacade. If a simple value is added to the component, it automatically becomes part of ComponentFacade as an attribute when it is created.
For example, if we add "color" to the cdsl grammar and it is read and reflected in the AST, when creating the corresponding 
```python
component = ComponentFacade(ast)
```
then,
```python
component.color
```
would contain the color value that has been read from the cdsl.

If we want to add more complex structures to the cdsl, although it is not mandatory, it may be convenient to create a subclass to contain such information.

Thus, we would have to create a class to which we would pass the value of this new attribute of the cdsl and specify in the mapping the name of the new key in the AST and the class that we have just created.

This way, when ComponentFacade is generated, this new attribute will be of the type of the class we have created.

If no specific class is specified on the mapping, the attribute will keep the same type and structure it has at the AST (list, dictionary, string)

# The Template classes
The Templates in robocompdsl represent the types of files that can be generated.
Specifically for a cdsl, you can generate code for a component in python or in c++. 
The templates of these files needed by the component can be found in the directory [templates](./templates).
 
## Usage of Python string.Template
To keep the template files clean in robocompdsl we have chosen to use the string.Template class of the standard library. You can find the specific documentation [here](https://docs.python.org/3/library/string.html#template-strings).

These files have the content that will have the final code file (static code) and some variables defined as ${variable} and that will be replaced later by other code pieces.

TODO: Talk about CustomTemplate class and indentation.

## AbstractTemplate class
In robocompdsl the class in charge of reading those source files and 
replacing the variables is located in [templates/common/abstracttemplate.py](./templates/common/abstracttemplate.py). 
In this module is defined the *AbstractTemplate* class from which any robocompdsl template should inherit.
The classes derived from this one must pass an instance of the ComponentFacade in the constructor.
 Finally, calling the generate_files method, the files corresponding to that component and template are generated.

## The *files* and *functions* directories
Given one of the two languages that can be generated, let's see for example for c++, the directory [templates/templateCPP](./templates/templateCPP/) contains several subdirectories:
* files: contains the template files as such. They will be the source files of the new component. These contain a static part that does not change and some variables that will be replaced by other pieces of code generated in a dynamic way.
* functions: The variables of the templates described above, must be filled with text/code generated according to the ComponentFacade that we are trying to convert into a component. 
Some of these code pieces to fill in the templates require some complex functions that are put in its own file in this directory, with the same name of the corresponding template file.

So, for example, we can find a template file in [templates/templateCPP/files/src/specificworker.cpp](./templates/templateCPP/files/src/specificworker.cpp)
and its corresponding function file in [templates/templateCPP/functions/src/specificworker_cpp.py](./templates/templateCPP/functions/src/specificworker_cpp.py).

Some template files do not have their corresponding function file. The reason for this is that they may not have variables in the template file or the functions to generate the code are very simple.

## How the Template variables code is generated
If we check the [string.Template](https://docs.python.org/3/library/string.html#string.Template.substitute) documentation we can see that one of the ways to give value 
to the variables contained in the text is calling the *substitute* method with a dictionary as parameter. The keys will be the names of the variables to be filled and the values, the text by which it is going to be replaced. 
Therefore, from the ComponentFacade and the variables to be filled, we can generate this dictionary. 
If you look into the code of the [__get_template_dict()](./templates/common/abstracttemplate.py#L150) method of [AbstracTemplate](./templates/common/abstracttemplate.py#L87), 
you can check that this process is done in 2 different ways:
1. First it check if the class that has inherited from AbstractTemplate has any method that 
is called the same as the name of the template but replacing the "/" and the "." with "_". 
This way, if we have the file src/commonbehaviorI.h, a method called src_commonbehaviorI_h would be expected.
This method has access through self.component to the corresponding ComponentFacade and it is expected to returns a dictionary
to be used with the Template.
Example: for the *src/mainUI.ui* file the method would be:
    ```python
    def src_mainUI_ui(self):
        return {
            'gui_type': self.component.gui.widget,
            'component_name': self.component.name
        }
    ```

2. If a method with that name is not found, the next thing that is done is to 
look in the functions directory for a file with the same path and the same name as the 
template but replacing the "." before the extension with a "_" and ending in this case in *.py*.
So, for example, for the template file/src/specificworker.h, the corresponding 
functions/src/specificworker_h.py will be searched.
Inside this file there will be probably several functions defined, but in order to work with the 
templates there must be a function called *get_template_dict* and that receives as parameter a 
component (ComponentFacade). It is expected to return a suitable dictionary for the replacement of the variables in the template file.

3. If none of the above conditions are met the __get_template_dict() method will return 
an empty dictionary. This situation is suitable for those template files that do not contain 
variables and that must simply be copied as they are to the component directory.

## Creating a new Template
If you want to create a new template to, for example, support a new language for the components,
what is expected is to replicate the same structure that can be found in 
[templateCPP](./templates/templateCPP) or [templatePython](./templates/templatePython). 
This new directory should have at the same time files of the template with the variables in a 
directory *files* and other directory *functions* with the corresponding files that could be necessary.
Finally, the template as such can be implemented as a module with a class that should inherit from 
AbstractTemplate. The necessary methods can be added to this new class as it has been described in the previous section for its use with the template.
You can consult the examples of this classes in the Python and C++ implementation of the templates:
* [templates/templateCPP/templatecpp.py](./templates/templateCPP/templatecpp.py)
* [templates/templatePython/templatepython.py](./templates/templatePython/templatepython.py)