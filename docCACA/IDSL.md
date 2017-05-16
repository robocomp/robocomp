## IDSL
IDSL(INterface Defenition Language is used to define interfaces in Robocomp). It is very close to the Zeroc ice IDSL. Similer to ice-IDSL it is very close to java and c++.

## Lexical Rules
IDSL permits comments of c and c++ style
IDSL uses a number of keywords, which must be spelled in lowercase. For example, class and dictionary are keywords and must be spelled as shown. Below is an list of keywords in IDSL 


| KeyWords        |            |   | |
| ------------- |:-------------:| -----:|-----:|
|bool|extends|string|byte|
|false|long|struct|float|
|module| throws|const|idempotent|
|true|dictionary|implements|void|
|double|int|out |enum|
|interface |sequence | exception|short|

Identifiers begin with an alphabetic character followed by any number of alphabetic characters or digits. Underscores are also permitted in identifiers with the following limitations:
* an identifier cannot begin or end with an underscore
* an identifier cannot contain multiple consecutive underscores

## Modules
IDSL provides the module construct to alleviate the pollution of global namespace.A module can contain any legal IDSL construct, including other module definitions. IDSL  requires all definitions to be nested inside a module, that is, you cannot define anything other than a module at global scope. Modules can be reopened in even in differnet files. Such reopening can be usefull for large projects. Modules map to a corresponding scoping construct in each programming language. For eg. scopes in c++ and packages in java.

## Types
It contains all the standred c++ data types - 
bool, byte,short,int,long,double,string

It supports c++ style __Enumerator__
	
    enum Fruit { Apple = 0, Pear = PearValue, Orange };

It supports c++ style __struct__, but they cant be nested. It can contain any of the basic types including enum.

	struct Point {
    	short x;
    	short y;
    };

    struct TwoPoints {
        Point coord1;
        Point coord2;
    };

### Sequence
Sequences are variable-length collections of elements.A sequence can be empty — that is, it can contain no elements, or it can hold any number of elements up to the memory limits of your platform. Sequences can contain elements that are themselves sequences. This arrangement allows you to create lists of lists:

	sequence<Fruit> FruitPlatter;

Sequences are used to model a variety of collections, such as vectors, lists, queues, sets, bags, or trees

### Dictionaries
A dictionary is a mapping from a key type to a value type.

	struct Employee {
	    long   number;
	    string firstName;
	    string lastName;
	};
	 
	dictionary<long, Employee> EmployeeMap;

The value type of a dictionary can be any IDSL type. However, the key type of a dictionary is limited to one of the following types:
* Integral types (byte, short, int, long, bool)
* string
* enum
* Structures containing only data members of legal key types

## Interfaces

IDSL interface defines the smallest grain of distribution Robocomp. For communication to take place, you must invoke operations on an components proxy which refers to an interface implimented in the component . To make the structure accessible, you must create an interface that allows clients to access the structure. A component may impliment one or more interfaces and each of those interface will have its a unique proxy.

You can think of an interface definition as the equivalent of the public part of a C++ class definition or as the equivalent of a Java interface, and of operation definitions as (virtual) member functions. Note that nothing but operation definitions are allowed to appear inside an interface definition. In particular, you cannot define a type, an exception, or a data member inside an interface.

### Operations

Operations are simply methods defined in the interface which can be invoked my the using an proxy to the interface.

An operation definition must contain a return type and zero or more parameter definitions. You must use void to indicate that an operation returns no value — there is no default return type for operations.

	interface CircadianRhythm {
    	void setSleepPeriod(TimeOfDay startTime, TimeOfDay stopTime);
    	// ...
	};
The parameters can be either input or output. As with input parameters, you can use multiple output parameters. If you have both input and output parameters for an operation, the output parameters must follow the input parameters.

	void changeSleepPeriod(TimeOfDay startTime, TimeOfDay stopTime, 
                    out TimeOfDay prevStartTime, out TimeOfDay prevStopTime);
 	
#### Overloading
IDSL does not support any form of overloading of operations.

#### Idempotent Operations

Idompotent Operations are operations that do not modify the state of the object they operate on. For example, x = 1; is an idempotent operation because it does not matter whether it is executed once or twice - either way, x ends up with the value 1. On the other hand, x += 1; is not an idempotent operation because executing it twice results in a different value for x than executing it once. Obviously, any read-only operation is idempotent
    
    interface Clock {
    	idempotent TimeOfDay getTime();
    	idempotent void setTime(TimeOfDay time);
	};
    
 The idempotent keyword is useful because it allows the Ice run time to be more aggressive when performing automatic retries to recover from errors. 


## Exceptions    

A user exception is much like a structure in that it contains a number of data members. However, unlike structures, exceptions can have zero data members, that is, be empty.You can specify a default value for an exception data member.

	exception RangeError {
      TimeOfDay errorTime;
      string reason = "out of range";
	};

Exceptions allow you to return an arbitrary amount of error information to the client if an error condition arises in the implementation of an operation. 

  	interface Clock {
    	idempotent TimeOfDay getTime();
      	idempotent void setTime(TimeOfDay time)
          throws RangeError, Error;
  	};
    
An operation can throw only those user exceptions that are listed in its exception specification. If, at run time, the implementation of an operation throws an exception that is not listed in its exception specification, the client receives a run-time exception

__Restrictions for User Exceptions__
* Exceptions are not first-class data types and first-class data types are not exceptions:
* You cannot pass an exception as a parameter value.
* You cannot use an exception as the type of a data member.
* You cannot use an exception as the element type of a sequence.
* You cannot use an exception as the key or value type of a dictionary.
* You cannot throw a value of non-exception type (such as a value of type int or string).

__User Exception Inheritance__

    exception ErrorBase {
        string reason;
    };

    exception RuntimeError extends ErrorBase {
        RTError err;
    };

## Inheritance