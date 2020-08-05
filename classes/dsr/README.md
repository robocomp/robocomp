# DSR (Deep State Representation)
CORTEX is a long term effort to build a series of architectural designs around the simple idea 
of a group of agents that share a distributed, dynamic representation acting as a working memory. 
This data structure is called **Deep State Representation (DSR)** due to the hybrid nature 
of the managed elements, geometric and symbolic, and concrete (laser data) and abstract (logical predicates).

![dsr-graph](https://user-images.githubusercontent.com/5784096/89400000-88576200-d713-11ea-8ac2-d2980568c2dc.png)
<sup>*The illustration shows a possible instance of the CORTEX architecture. Thecentral part of the ring contains the DSR graph that is shared by all agents, from whoma reference implementation is presented here. Coloured boxes represent agents provid-ing different functionalities to the whole. The purple box is an agent that can connectto the real robot or to a realistic simulation of it, providing the basic infrastructure toexplore prediction and anticipation capabilities*</sup>

This documentation describes the classes that allow the creation of agents that use this Deep State Representation.

## DSR-API (aka G-API)
G-API is the user-level access layer to G. It is composed by a set of core methods that access the underlying CRDT and RTPS APIs, and an extendable  set of auxiliary methods added to simplify the user coding tasks. 


The most important features of the G-API are:

-   It always works on a copy a node. The obtention of the copy is done by a core method using shared mutex technology. Once a copy of the node is returned, the user can edit it for as long as she wants. When it is ready, the node is reinserted in G using another core method. This feature makes the API thread-safe.
    
-   There are a group of methods, that include the word local in their name, created to change the attributes of a node. These methods do not reinsert the node back into G and the user is left with this responsibility.
    
-   DSRGraph has been created as a QObject to emit signals whenever a node is created, deleted or modified. Using this functionality, a set of graphic classes have been created to show in real-time the state of G. These classes can be connected at run-time to the signals. There is an abstract class from which all of them inherit that can be used to create more user-defined observers of G.
    
-   To create a new node, a unique identifier is needed. To guarantee this requirement, the node creation method places a RPC call to the special agent idserver, using standard RoboComp communication methods. Idserver returns a unique id that can be safely added to the new node.
    

- G can be serialized to a JSON file from any agent but it is better to do it only from the idserver agent, to avoid the spreading of copies of the graph in different states.

<!--stackedit_data:
eyJoaXN0b3J5IjpbMTE4NTI5OTkzM119
-->