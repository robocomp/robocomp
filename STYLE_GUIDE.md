#Robocomp Style Guide

<!-- markdown="1" is required for GitHub Pages to render the TOC properly. -->

<details markdown="1">
  <summary>Table of Contents</summary>

- [Robocomp Python Style Guide](#robocomp-python-style-guide)
- [Robocomp C++ Style Guide](#robocomp-c---style-guide)
  * [Layout](#layout)
    + [Indentation](#indentation)
    + [Added spaces](#added-spaces)
    + [The #define Guard](#the--define-guard)
  * [Naming Conventions](#naming-conventions)
    + [Variables](#variables)
    + [Functions](#functions)
    + [Classes](#classes)
    + [Methods](#methods)
    + [Curly Braces](#curly-braces)
    + [Always use namespaces](#always-use-namespaces)
    
</details>


## Robocomp Python Style Guide
For Python it is very simple. We try to follow the style code defined in [PEP 8 -- Style Guide for Python Code](https://www.python.org/dev/peps/pep-0008/).

If you find python code in Robocomp that does not comply with PEP8 we would appreciate a Pull Request to fix it. Thank you.


## Robocomp C++ Style Guide

For C++ it is not so simple. There is no standard style defined. Several projects have established their own style codes. In Robocomp we are trying to agree and define a style code and for now what we have are a series of recommendations:

### Layout

#### Indentation
4 spaces. No tabs.

#### Added spaces
Space before each new parameter in function calls and function declarations

#### The #define Guard
https://google.github.io/styleguide/cppguide.html#The__define_Guard


### Naming Conventions

#### Variables
Use snake_case

#### Functions
Use snake_case

#### Classes
Use CamelCase

#### Methods
Use snake_case

#### Curly Braces
New line before open brace

#### Always use namespaces
**Don’t use the using clause for namespaces** in .h files
https://lefticus.gitbooks.io/cpp-best-practices/content/03-Style.html#always-use-namespaces
