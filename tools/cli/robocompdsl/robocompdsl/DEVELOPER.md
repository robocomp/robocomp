
RoboCompDSL's Developer Guide
=============================

# Table of Contents
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
  * [ComponentTemplatesManager class](#componenttemplatesmanager-class)
  * [The *files* and *functions* directories](#the--files--and--functions--directories)
  * [How the Template variables code is generated](#how-the-template-variables-code-is-generated)
  * [Creating a new Template](#creating-a-new-template)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Introduction

This is the developers documentation. It explains some of the concepts needed to understand robocompdsl's code, to modify it and to add new features.  

<small><i>Note: If your are looking for documentation about the usage of RobocompDSL as a tool you can find it in the main [README.md](../README.md) file.</i></small>
# Main Concepts
RobocompDSL is a tool to generate components and interface files using a custom DSL (Domain Specific Languages). 
It is mainly used to generate the basic structure of a component in C++ or Python, from its definition in a .cdsl file.
We could summarize that the main function of robocompdsl would be:
```
.cdsl file ==> (YET TO EXPLAIN) ==> Component (many files)
```

The first step to achieve this goal is to be able to read a .cdsl file (which has its own grammar) and convert it to an AST (Abstract Syntax Tree) that we can use in python. This is what PyParsing is being used for.

Pyparsing generates its own data structures, _PyParsingResult_, which can be accessed as an array, dictionary or class, as needed.
But it has been decided that for convenience and to control the access to the information of this structure, it should be converted to a specific robocompdsl class: _ComponentFacade_. This class can be generated directly from what PyParsing returns and it allows us, through a small hierarchy of classes, to access in a comfortable and ordered way to the information that we will require from the component that we have just read.

In robocompdsl, you need parsers for several file types (.cdsl, .idsl, .smdsl). For this reason, 
PyParsing has been encapsulated behind DSLFactory (we will talk about it later) which provides 
the necessary parsers for different file types. Considering this intermediate data structures, the process looks like:
```
.cdsl file ==> Parser ==> AST ==> ComponentFacade ==> (YET TO EXPLAIN) ==> Component (many files).
```

Currently, robocompdsl can generate components in c++ and python. The files needed to generate these components are found as Templates, which are designed to receive a ComponentFacade type object and generate the final component files from it.
So finally we have that the process that is followed internally to pass from a .cdsl to a Component would be this one:
```
.cdsl file ==> Parser ==> AST ==> ComponentFacade ==> Template ==> Component (many files).
```

# DSLFactory
As mentioned above, robocompdsl must be able to read several file types, each with its own syntax. 
Currently, you can read .cdsl (Component Domain Specific Language), .idsl (Interface Domain Specific Language), 
and .smdsl (State Machine Domain Specific Language) files.
Each of these 3 file types requires a specific parser, but the operations are very similar.
First, the content of the file is loaded, and then a parser is applied to it to check the syntax and obtain an 
AST ([Abstract Sintaxt Tree](https://en.wikipedia.org/wiki/Abstract_syntax_tree)).

_DSLFactory_ ([dsl_parsers/dsl_factory.py](dsl_parsers/dsl_factory.py)) was designed to perform 
these tasks. This class determines which type of file it is processing and calls the appropriate 
parser to finally return the AST corresponding to the dsl file read.
The different parsers can be found in [dsl_parsers/specific_parsers/](./dsl_parsers/specific_parsers/).

DSLFactory was created to work as a  [Singleton](https://en.wikipedia.org/wiki/Singleton_pattern), so in the
whole execution of  robocompdsl, there can only be one instance of it and it will be shared in all the code
points where it is used.
If we add to these a rudimentary cache system that stores the structure that is generated for each parsed file path, 
we have that even if we need several times the content of a dsl file, it will only be parsed 
the first time it is required.

Therefore, the main tasks carried out by _DSLFactory_ are:
* Reading CDSL files
* Creating the appropriate parser for each file type
* Using the parser to generate a structure (a _ComponentFacade_)
* Cache the file path and the generated structure


## DSL Parsers

DSL parsers are in charge of defining the grammar for one of the file types that robocompdsl 
can read and generate the [AST](https://en.wikipedia.org/wiki/Abstract_syntax_tree) from a 
string and that grammar.

To unify and simplify the implementation of new parsers, the DSLParserTemplate abstract class was 
created ([dsl_parsers/dsl_parser_abstract.py](dsl_parsers/dsl_parser_abstract.py)). This class 
implements some of the functionalities common to all parsers and forces the implementation of the
two required methods.

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

Essentially, PyParsing offers a series of classes and operators that allow us to to build parsers by describing grammars.

Every parser has a parseString method to obtain the AST from the text it receives as a parameter. This AST comes in the form of PyParsingResult nested classes.

#### Pyparsing Parser Creation
If you are going to create your own parser you can check the _create_parser() methods for 
[cdsl](dsl_parsers/specific_parsers/cdsl/cdsl_parser.py#L17), [idsl](dsl_parsers/specific_parsers/idsl_parser.py#L15) and [smdsl](dsl_parsers/specific_parsers/smdsl_parser.py#L14) in these links.

You can also consult other less specific examples of the use of PyParsing in its own [repository](https://github.com/pyparsing/pyparsing/tree/master/examples).

# ComponentFacade: AST to Component Object
In the specific case of the cdsl, given the complexity of the structure generated and 
the need to unify the way to access it, it was decided to create a class from the AST 
returned by PyParsing and that would contain all the necessary information regarding 
that specific component read from the file. 

This class is ComponentFacade ([dsl_parsers/specific_parsers/cdsl/componentfacade.py](dsl_parsers/specific_parsers/cdsl/componentfacade.py)).

The main purpose of this class is to extend the functionality of the dictionary 
that represents the AST. When this class is instantiated you can pass the AST directly to it
 in the constructor. Each key of that dictionary will become an attribute of the
 ComponentFacade class and they are assigned the value that they contain in the dictionary.
However, some of these keys are mapped so that their value is converted to another 
specific class. This mapping of keys to class types, as well as the subclasses of ComponentFacade itself,
 can be found in the file [dsl_parsers/specific_parsers/cdsl/componentfacade.py](dsl_parsers/specific_parsers/cdsl/componentfacade.py).

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
    def is_agm_agent(self):
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
The templates of these files needed by the component can be found in the directory [templates](templates).
 
## Usage of Python string.Template
To keep the template files clean in robocompdsl we have chosen to use the string.Template class of the standard library. You can find the specific documentation [here](https://docs.python.org/3/library/string.html#template-strings).

These files have the content that will have the final code file (static code) and some variables defined as ${variable} and that will be replaced later by other code pieces.

TODO: Talk about CustomTemplate class and indentation.

## ComponentTemplatesManager class
In robocompdsl the class in charge of reading those source files and 
replacing the variables is located in [templates/common/abstracttemplatesmanager.py](templates/common/abstracttemplatesmanager.py). 
In this module is defined the *ComponentTemplatesManager* class from which any robocompdsl template should inherit.
The classes derived from this one must pass an instance of the ComponentFacade in the constructor.
 Finally, calling the generate_files method, the files corresponding to that component and template are generated.
 
*TODO: Add documentation of the methods _pre_generation_check, _output_file_rename, _post_generation_action*  

*TODO: Add documentation of the self.files dict.*

## The *files* and *functions* directories
Given one of the two languages that can be generated, let's see for example for c++, the directory [templates/templateCPP](./templates/templateCPP/) contains several subdirectories:
* files: contains the template files as such. They will be the source files of the new component. These contain a static part that does not change and some variables that will be replaced by other pieces of code generated in a dynamic way.
* functions: The variables of the templates described above, must be filled with text/code generated according to the ComponentFacade that we are trying to convert into a component. 
Some of these code pieces to fill in the templates require some complex functions that are put in its own file in this directory, with the same name of the corresponding template file.

So, for example, we can find a template file in [templates/templateCPP/files/src/specificworker.cpp](templates/templateCPP/files/src/specificworker.cpp)
and its corresponding function file in [templates/templateCPP/functions/src/specificworker_cpp.py](./templates/templateCPP/functions/src/specificworker_cpp.py).

Some template files do not have their corresponding function file. The reason for this is that they may not have variables in the template file or the functions to generate the code are very simple.

## How the Template variables code is generated
If we check the [string.Template](https://docs.python.org/3/library/string.html#string.Template.substitute) documentation we can see that one of the ways to set the variables contained in the text is calling the *substitute* method with a dictionary as parameter. The keys will be the names of the variables to be filled and the values, the text by which it is going to be replaced. 
Therefore, from the ComponentFacade and the variables to be filled, we can generate this dictionary. 
If you look into the code of the [\_\_get_template_dict()](templates/common/abstracttemplatesmanager.py#L150) method of [AbstracTemplate](templates/common/abstracttemplatesmanager.py#L87), 
you can check that this process is done in 2 different ways:
1. First it checks if the class that has inherited from ComponentTemplatesManager has any method that 
is called as the template but replacing the "/" and the "." with "\_". 
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
template but replacing the "." before the extension with a "\_" and ending in this case in *.py*.
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
[templateCPP](templates/templateCPP) or [templatePython](templates/templatePython). 
This new directory should have at the same time files of the template with the variables in a 
directory *files* and other directory *functions* with the corresponding files that could be necessary.
Finally, the template as such can be implemented as a module with a class that should inherit from 
ComponentTemplatesManager. The necessary methods can be added to this new class as it has been described in the previous section for its use with the template.
You can consult the examples of this classes in the Python and C++ implementation of the templates:
* [templates/templateCPP/templatecpp.py](templates/templateCPP/templatecpp.py)
* [templates/templatePython/templatepython.py](templates/templatePython/templatepython.py)


# Testing
In robocompdsl two types of tests can be distinguished. Automatic tests that are executed to check that everything is still working when changes are made, and tests that can be performed manually to check that code generation works as expected.

## Automatic tests
The automatic tests are executed with python unittest and the code can be found in the test folder of robocompdsl.

Among the automatic tests, there are some in particular that compare the recently generated code for some components that are used as reference with the code that previously was expected to be generated. This type of test is especially useful when you want to make changes in the operation of robocompdsl but it is expected that the generated code remains the same.
On the other hand, this has the disadvantage that when new features are added to robocompdsl these already generated files that are used as reference must be updated, once it is clear that the new generation works correctly.

In test/resources/reference_components you can find the code of the components against which the new generation of code is compared.

The automatic tests are also executed as part of robocomp's Continuous Integration (CI) process and their results can be seen here: http://robocomp-ci.unex.es

The manual part of the robocompdsl tests is found in autogeneration_tests/test_cdsl.
In this directory you can find a script called test_component_generation.py and a series of subdirectories with .cdsl and .smdsl files. The idea is that through test_component_generation.py you can trigger the generation, compilation and startup test of the components defined in all subdirectories. Once this process is finished, a summary of how it went is shown.

Lo más habitual es que el resultado de la ejecución de este script pueda ser utilizado como componentes de referencia en los tests automáticos.
Si ejecuta el script de la siguiente manera:
python3 test_component_generation.py -dg

Se generarán los ficheros de cada componente y no se borrarán para que pueda copiarlos o comparar con los de referencia. 
Puede por ejemplo usar meld para ver que cambios ha habido entre la nueva generación y los de referencia:
python3 test_component_generation.py -dgf dsr

Most often the results of the execution of this script can be used as reference components in the automatic tests.
If you run the script in the following way:
```shell script
python3 test_component_generation.py -dg
```
the files for each component will be generated and will not be deleted so you can copy them or compare them with the reference ones. 
You can for example use meld to see what changes have occurred between the new generation and the reference ones:
```shell script
meld . ../../test/resources/reference_components/
```
This way you can correct whatever is necessary from the generation or update the reference components.

It is recommended that once you are done with the manual tests you make sure to run them:
```shell script
python3 test_component_generation.py -c
```
to delete temporary files and not upload them to the repository.