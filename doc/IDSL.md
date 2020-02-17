# IDSL

IDSL is  Interface Definition Language used to create and define interfaces in Robocomp. It is very close to Zeroc Ice's IDSL. 

## Syntax

- IDSL permits comments in C and C++ style, starting with `//`.

- IDSL uses a number of keywords, spelled in lowercase. For example, class and dictionary are keywords and must be written exactly as shown. Below is an list of keywords in IDSL: 


| KeyWords        |            |   | |
| :-------------: |:-------------:|:-----:|:-----:|
|bool|extends|string|byte|
|false|long|struct|float|
|module| throws|const|idempotent|
|true|dictionary|implements|void|
|double|int|out |enum|
|interface |sequence | exception|short|

- Identifiers begin with an alphabetic character, followed by any number of alphabetic characters or digits. Underscores are also permitted in identifiers with the following limitations:
    * an identifier cannot begin or end with an underscore.
    * an identifier cannot contain multiple consecutive underscores.

### Modules

As in Zeroc Ice [[1](https://doc.zeroc.com/ice/3.6/the-slice-language/modules)], modules are required, and used to prevent pollution of the global namespace. Modules have the following characteristics:

- They can contain any legal IDSL construct, including other module definitions. 

- IDSL requires all definitions to be nested inside a module, that is, you cannot define anything other than a module at global scope.

- They can be reopened, even in different files. Such reopening can be useful for large projects. Modules map to a corresponding scoping construct in each programming language. For example, scopes in C++ and packages in Java.

### Types

It contains all standard C++ data types, such as,` bool`, `byte`, `short`, `int`, `long`, `double`, `string`.

It supports C++ style __Enumerator__.
    
    enum Fruit { Apple = 0, Pear = PearValue, Orange };

It supports C++ style __struct__, but they __can't__ be nested. It can contain any of the basic types including enum.

```cpp
struct Point {
        short x;
        short y;
};

struct TwoPoints {
	Point coord1;
        Point coord2;
};
```

### Sequence

Sequences are variable-length collections of elements. A sequence can be empty, or it can hold any number of elements up to the memory limits of your platform. Sequences can contain elements that are themselves sequences. This arrangement allows you to create lists of lists:

    sequence<Fruit> FruitPlatter;

Sequences are used to model a variety of collections, such as vectors, lists, queues, sets, bags, or trees.

### Dictionaries

A dictionary is a mapping from a key type to a value type.

```cpp
struct Employee {
    long number;
    string firstName;
    string lastName;
};
     
dictionary<long, Employee> EmployeeMap;
```

The value type of a dictionary can be any IDSL type. However, the key type of a dictionary is limited to one of the following types:
* Integral types (byte, short, int, long, bool).
* string
* enum
* Structures containing only data members of allowed key types.

## Interfaces

IDSL interface is the core of what makes up Robocomp. 

For communication between components to take place, you must invoke operations on a component's proxy, which refers to an interface implemented in the component. To make the structure accessible, you must create an interface that allows clients to access the structure. 

A component may implement one or more interfaces and each of those interfaces will have its unique proxy.

You can think of an interface definition as the equivalent of the public part of a C++ class definition, or a Java interface, and operation definitions as (virtual) member functions. 

Note that nothing but operation definitions are allowed to appear inside an interface definition. In particular, you cannot define a type, an exception, or a data member inside an interface.

### Operations

Operations are simply methods defined in the interface which can be invoked using a proxy to the interface.

An operation definition must contain a return type and parameter definitions (it can also be left empty). You must use void to indicate that an operation returns no value — there is no default return type for operations.

```cpp
interface CircadianRhythm {
    	void setSleepPeriod(TimeOfDay startTime, TimeOfDay stopTime);
    	// ...
};
```

The parameters can be either input or output. As with input parameters, you can use multiple output parameters. If you have both input and output parameters for an operation, the output parameters must follow the input parameters.

```cpp
void changeSleepPeriod(TimeOfDay startTime, TimeOfDay stopTime, 
	out TimeOfDay prevStartTime, out TimeOfDay prevStopTime);
```
 
#### Overloading

IDSL does not support any form of overloading of operations.

#### Idempotent Operations

Idempotent Operations are operations that do not modify the state of the object they operate on. For example, `x = 1;` is an idempotent operation because it does not matter whether it is executed once or twice - either way, x ends up with the value 1. On the other hand, `x += 1;` is not an idempotent operation because executing it twice results in a different value for x than executing it once. By definition any read-only operation is idempotent.

```cpp
interface Clock {
	idempotent TimeOfDay getTime();
	idempotent void setTime(TimeOfDay time);
};
```
    
The idempotent keyword is useful because it allows the Ice compiler to be more aggressive when performing automatic retries to recover from errors. 

### Exceptions    

You can specify a default value for an exception data member.

```cpp
exception RangeError {
      TimeOfDay errorTime;
      string reason = "out of range";
};
```

Exceptions allow you to return an arbitrary amount of error information to the client if an error condition arises in the implementation of an operation. 

```cpp
interface Clock {
	idempotent TimeOfDay getTime();
      	idempotent void setTime(TimeOfDay timep)
        	throws RangeError, Error;
};
```
    
An operation can throw only those user exceptions that are listed in its exception specification. If at run time, the implementation of an operation throws an exception that is not listed in its exception specification, the client receives a run-time exception.

#### Restrictions for User Exceptions

* Exceptions are not first-class data types and first-class data types are not exceptions.
* You cannot pass an exception as a parameter value.
* You cannot use an exception as the type of a data member.
* You cannot use an exception as the element type of a sequence.
* You cannot use an exception as the key or value type of a dictionary.
* You cannot throw a value of non-exception type (such as a value of type int or string).

#### Example of User Exception Inheritance

```cpp
exception ErrorBase {
	string reason;
};

exception RuntimeError extends ErrorBase {
        RTError err;
};
```

## Inheritance
