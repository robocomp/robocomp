# DSR (Deep State Representation)
- [Description](#description)
- [Definitions](#definitions)
- [Dependencies and Installation](#dependencies-and-installation)
  * [Installing agents](#installing-agents)
- [Developer Documentation](#developer-documentation)
  * [DSR-API (aka G-API)](#dsr-api--aka-g-api-)
  * [CORE](#core)
  * [Auxiliary sub-APIs](#auxiliary-sub-apis)
    + [RT sub-API](#rt-sub-api)
      - [Overloaded method using move semantics.](#overloaded-method-using-move-semantics)
    + [IO sub-API](#io-sub-api)
    + [Innermodel sub-API](#innermodel-sub-api)
  * [CRDT- API](#crdt--api)
  * [Node struct](#node-struct)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Description

CORTEX is a long term effort to build a series of architectural designs around the simple idea 
of a group of agents that share a distributed, dynamic representation acting as a working memory. 
This data structure is called **Deep State Representation (DSR)** due to the hybrid nature 
of the managed elements, geometric and symbolic, and concrete (laser data) and abstract (logical predicates).
This software is the new infrastructure that will help in the creation of CORTEX instances. A CORTEX instance is a set of software components, called _agents_, that share a distributed data structured called (_G_)raph playing the role of a working memory. Agents are C++ programs that can be generated using RoboComp's code generator, _robocompdsl_.  Agents are created with an instance of the FastRTPS middleware (by eProsima) configured in reliable multicast mode. A standard graph data structure is defined using FastRTPS's IDL and converted into a C++ class that is part of the agent's core. This class is extended using the _delta mutators_ [CRDT](https://github.com/CBaquero/delta-enabled-crdts) code kindly provided by Carlos Baquero. With this added functionality, all the copies of G held by the participating agents acquire the property of _eventual consistency_. This property entails that all copies, being different at any given moment, will converge to the exact same state in a finite amount of time, after all agents stop editing the graph. With _delta mutators_, modifications made to G by an agent are propagated to all others and integrated in their respective copies. This occurs even if the network cannot guarantee that each packet is delivered exactly once, _causal broadcast_. Note that if operations on the graph were transmitted instead of changes in state, then _exactly once_ semantics would be needed.  Finally, the resulting local graph _G_ is used by the developer through a carefully designed thread safe API, _G_API_.

<img src="https://user-images.githubusercontent.com/5784096/90373871-e3257d80-e072-11ea-9933-0392ea9ae7f1.png" width="800">

<sup>*The illustration shows a possible instance of the CORTEX architecture. The central part of the ring contains the **DSR graph** that is shared by all agents, from whom a reference implementation is presented here. Coloured boxes represent agents providing different functionalities to the whole. The purple box is an agent that can connect to the real robot or to a realistic simulation of it, providing the basic infrastructure to explore prediction and anticipation capabilities*</sup>

Conceptually, the DSR represents a network of entities and relations among them. Relations can be unary or binary predicates, while the entities may have complex numeric properties such as pose transformation matrices that represent the kinematic relations of objects in the world and the robot’s parts. Mathematically, the DSR is internalized as a directed graph with attributed edges. As a hybrid representation that stores information at both geometric and symbolic levels, the nodes of the DSR store concepts that can be symbolic, geometric or a combination of both. Metric concepts describe numeric quantities of objects in the world, which can be structures such as a three-dimensional mesh, scalars such as the mass of a link, or lists such as revision dates. Edges represent relationships between nodes. Two nodes may have several kinds of relationships but only one of them can be geometric. The geometric relationship is expressed with a fixed label called *RT*. This label stores the transformation matrix (expressed as a rotation-translation) between them.

# Definitions
- [CRDT](https://crdt.tech/). "A Conflict-free Replicated Data Type (CRDT) is a data structure that simplifies distributed data storage systems and multi-user applications.". [Video](https://www.youtube.com/watch?v=oyUHd894w18) explanation.  
[![CRDT explained](https://img.youtube.com/vi/oyUHd894w18/0.jpg)](https://www.youtube.com/watch?v=oyUHd894w18)
- CORTEX, a Robotics Cognitive Architecture based on the idea of a set of software modules sharing a common representation or working memory. 
- G, a distributed graph used as the CORTEX working memory. It exists as the set of local copies held by all the participating components in a CORTEX configuration
- DSR Agent (DA), is a C++ RoboComp component extended with the functionality to edit the distributed graph G.
- Node, a vertex of the graph G.
- Edge, a directional edge connecting to nodes. Edges can have attributes.
- All nodes and edges have type. The list of possible types is shown below.
- Attribute, a property of a node or an edge. Attributes can be of any of these types
  1. string
  2. int (signed 32)
  3. float
  4. vector<float>
  5. bool
  6. in V1.1 vector<byte>
  All attributes have a name and in V1.1 will hava a timestamp with the last modification time
  
- Attributes' names must be taken from a predefined list of tokens with types, for example ('name', string)('pos_x', float). A complete lists is provided below.
- _RT edges_ are edges of type _RT_ that code a geometric relation between two nodes. RT edges have two atributes _rotation_euler_xyz_ and _translation_ that hold    the euler angles and translation coordinates of the R|t 4x4 matrix that represents a rotation followed by a translation pose of the child with respect to the parent's reference frame. There are methods in the G-API to directly recover the Rt matrix from the edge as a RoboComp RTMat.
- There is a singular node in G named, "world", that representes the origin of the current reference frame for the robot. This node is the root of a kinematic tree       linked by RT edges. This tree is embedded in G and represent the set of physical elements believed to exist in the world. This tree is called _innermodel_ and can be automatically drawn in 3D using OpenSceneGraph and in 2D using Qt. Both representation are included in the UI off all generated agents.
- An agent generated with _robocompdsl_ includes the object _G_ that can be accessed using its public API

> This documentation describes the classes that allow the creation of agents to use this Deep State Representation.

# Installation

## Dependencies

It's assumed that you have already installed [robocomp](https://github.com/robocomp/robocomp/blob/development/README.md#installation-from-source).    

__IMPORTANT__: DSR is only supported in Ubuntu 20.04. We can't help with the issues of other distros or versions.  

To be able to use the DSR/CORTEX infraestructure you need to follow the next steps:

### Step 1
From ubuntu repositories you need:
```bash
sudo apt install libasio-dev libtinyxml2-dev libopencv-dev libqglviewer-dev-qt5 libeigen3-dev python3-dev python3-pybind11 cmake gcc-10 g++-10
```

> __NOTE :__ If you are using `python` with `Anaconda`, `cmake` might not be able to find pybind11 installation. So, you have to install it using `conda-forge` as well :
> ```bash
> conda install -c conda-forge pybind11
> ```


You need to update the alternatives for g++ and gcc:
```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 1
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 1
```
then
```bash
sudo update-alternatives --config gcc
```
and select version 10. Do the same for g++.


### Step 2
You need the following third-party software:

- cppitertools
```sh
      sudo git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
      cd /usr/local/include/cppitertools
      sudo mkdir build
      cd build
      sudo cmake ..
      sudo make install
```

- Fast-RTPS (aka Fast-DDS) from https://www.eprosima.com/. Follow these steps:
```sh
      mkdir software
      cd software
      git clone https://github.com/eProsima/Fast-CDR.git
      mkdir Fast-CDR/build 
      cd Fast-CDR/build
      export MAKEFLAGS=-j$(($(grep -c ^processor /proc/cpuinfo) - 0))
      cmake ..
      cmake --build . 
      sudo make install 
      
      cd ~/software
      git clone https://github.com/eProsima/foonathan_memory_vendor.git
      cd foonathan_memory_vendor
      mkdir build 
      cd build
      cmake ..
      cmake --build . 
      sudo make install 
      cd ~/software
      
      git clone https://github.com/eProsima/Fast-DDS.git
      cd Fast-DDS
      mkdir build 
      cd build
      cmake ..
      cmake --build . 
      sudo make install
```

Update the system cache of dynamic libraries with 

```sh
sudo ldconfig
```

## Installation
Next step is to compile and install the DSR libs. You need to go to ~/robocomp/
and execute this:
```bash
mkdir build
cd build
cmake -DDSR=TRUE ..
make -j$(nproc)
sudo make install
sudo ldconfig
```


## Common Issues

1)  __DSR compilation requires GCC 9+, while other components might require GCC 8 or older (Ubuntu 20.04) :__
    -   Install multiple C and C++ compiler versions :
        ```bash
        sudo apt install build-essential
        sudo apt -y install gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9 g++-10 gcc-10
        ```
    -   Use the `update-alternatives` tool to create list of multiple GCC and G++ compiler alternatives :
        ```bash
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
        sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
        sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
        sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 1
        sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 1
        ```
    -   Check the available C and C++ compilers list on your system and select desired version by entering relevant selection number :
        ```bash
        sudo update-alternatives --config gcc
        sudo update-alternatives --config g++
        ```

3)  __This application failed to start because no Qt platform plugin could be initialized :__
    -   This problem can appear when trying to start `viriatoPyrep`, due to compatibility issues with _Qt_ version in _OpenCV_ and _VREP_.

    -   This problem is solved by installing `opencv-python-headless` :
        ```bash
        pip install opencv-python-headless
        ```


## Installing agents
If you want to install the existing agents you can clone the [dsr-graph](https://github.com/robocomp/dsr-graph) repository and read the related documentation.


# Developer Documentation
## DSR-API (aka G-API)
G-API is the user-level access layer to G. It is composed by a set of core methods that access the underlying CRDT and RTPS APIs, and an extendable  set of auxiliary methods added to simplify the user coding tasks. 


The most important features of the G-API are:

-   It always works on a copy a node. The obtention of the copy is done by a core method using shared mutex technology. Once a copy of the node is returned, the user can edit it for as long as she wants. When it is ready, the node is reinserted in G using another core method. This feature makes the API thread-safe.
    
-   There are a group of methods, that include the word local in their name, created to change the attributes of a node. These methods do not reinsert the node back into G and the user is left with this responsibility.
    
-   DSRGraph has been created as a QObject to emit signals whenever a node is created, deleted or modified. Using this functionality, a set of graphic classes have been created to show in real-time the state of G. These classes can be connected at run-time to the signals. There is an abstract class from which all of them inherit that can be used to create more user-defined observers of G.
    
-   To create a new node, a unique identifier is needed. To guarantee this requirement, the node creation method places a RPC call to the special agent idserver, using standard RoboComp communication methods. Idserver returns a unique id that can be safely added to the new node.

-   G can be serialized to a JSON file from any agent but it is better to do it only from the idserver agent, to avoid the spreading of copies of the graph in different states.


## Common examples
Before we start with the detailed description of the DSR API methods, some examples of common usage are shown:
```c++
// We create a shared pointer and get the DSRGraph
auto G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes



/*========================================================*/
/*======== Getting a node and updating attributes ========*/

// Some calculations are done
zVel = (adv_conv * QVec::vec2(zVel,1.0))[0];  
rotVel = (rot_conv * QVec::vec2(rotVel,1.0))[0];  
xVel = (side_conv * QVec::vec2(xVel, 1.0))[0];

// Get the graph node of the robot
auto robot_node = G->get_node(robot_name);  

// Modify the robot node attributes locally
G->add_or_modify_attrib_local<ref_adv_speed>(robot_node.value(), (float)xVel);  
G->add_or_modify_attrib_local<ref_rot_speed>(robot_node.value(), (float)rotVel);  
G->add_or_modify_attrib_local<ref_side_speed>(robot_node.value(), (float)zVel);  

// Update the node in the graph to set the modified attributes available
G->update_node(robot_node.value());



/*==================================================================*/
/*======== Using RT API, getting edge, modifying attributes ========*/

// Get a pointer to th rt API
auto rt = G->get_rt_api();

// Use it to get an RT Edge from the graph
auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();  

// Modify the attributes locally
G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0, bState.alpha});  
G->modify_attrib_local<rt_translation_att>(edge, std::vector<float>{bState.x, bState.z, 0.0});  
G->modify_attrib_local<robot_current_linear_speed_att>(edge, std::vector<float>{bState.advVx, 0, bState.advVz});
G->modify_attrib_local<robot_current_angular_speed_att>(edge, std::vector<float>{0, 0, bState.rotV});

// Update the edge in the graph to set the modified attributes available
G->insert_or_assign_edge(edge);



/*=================================================*/
/*======== Other usefull methods available ========*/

// Save the current state of the graph in a json file.
G->write_to_json_file("./"+agent_name+".json");  

//Reset the graph
G.reset();
```

## Predefined names and types
To avoid the creation or modification of attributes by mistake, a predefined and expandable list of attributes that can be used in nodes and links has been created. You can see the definition of all these attributes and their type in the file:
[core/types/type_checking/dsr_attr_name.h](https://github.com/robocomp/robocomp/blob/development/libs/dsr/core/include/dsr/core/types/type_checking/dsr_attr_name.h#L80).

In the same way you can see the predefined types of nodes and links in the following files:
[core/types/type_checking/dsr_node_type.h](https://github.com/robocomp/robocomp/blob/development/libs/dsr/core/include/dsr/core/types/type_checking/dsr_node_type.h#L15)
and
[core/types/type_checking/dsr_edge_type.h](https://github.com/robocomp/robocomp/blob/development/libs/dsr/core/include/dsr/core/types/type_checking/dsr_edge_type.h#L15).

## CORE

```c++
std::optional<Node> get_node(const std::string& name);
```
- Returns a node if it exists in G. *name* is one of the properties of Node.  

&nbsp;
       
```c++
std::optional<Node> get_node(int id);
```
- Returns a node if it exists in G. id is one of the main properties of Node

&nbsp;
```c++
bool update_node(const Node& node);
```
- Updates the attributes of a node with the content of the one given by parameter.
Returns true if the node is updated, returns false if node doesn’t exist or fails updating.

&nbsp;
```c++
bool delete_node(const std::string &name);
```
- Deletes the node with the name given by parameter.
Returns true if the node is deleted, returns false if node doesn’t exist or fails updating.

&nbsp;
```c++
bool delete_node(int id);
```
- Deletes the node with the name given by parameter.
Returns true if the node is deleted, returns false if node doesn’t exist or fails updating.

&nbsp;
  
```c++
std::optional<uint32_t> insert_node(Node& node);
```
- Inserts in the Graph the Node given by parameter.
Returns optional with the id of the Node if success, otherwise returns a void optional.

&nbsp;
  
```c++
std::optional<Edge> get_edge(const std::string& from, const std::string& to, const std::string& key);
```
- Returns an optional with the Edge going from node named from to node named to and key ??

&nbsp;

  
```c++
std::optional<Edge> get_edge(int from, int to, const std::string& key);
```
- Returns an optional with the Edge that goes from node id to node id from and key ??

&nbsp;
  
```c++
std::optional<Edge> get_edge(const Node &n, int to, const std::string& key)
```
- Returns an optional with the Edge that goes from current node to node id from and key ??.

&nbsp;
  
```c++
bool insert_or_assign_edge(const Edge& attrs);
```
  
&nbsp;
```c++
bool delete_edge(const std::string& from, const std::string& to , const std::string& key);
```
- Deletes the edge going from node named from to node named to and key ??.
Returns true if the operation completes successfully.

&nbsp;

  
```c++
bool delete_edge(int from, int t, const std::string& key);
```
- Deletes edge going from node id from to node id to and with key key.
Returns true if the operation completes successfully.

&nbsp;

  

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  
```c++
std::optional<Node> get_node_root();
```
- Returns the root node as defined in idserver or void if not defined

&nbsp;
  
```c++
std::vector<Node> get_nodes_by_type(const std::string& type);
```
- Returns a vector of nodes whose type is type

&nbsp;

  
```c++
std::optional<std::string> get_name_from_id(std::int32_t id);
```
- Returns an optional string with the name of node defined by its id

&nbsp;

  
```c++
std::optional<int> get_id_from_name(const std::string &name);
```
- Returns an optional string with the id (int) of node defined by its name

&nbsp;

  
```c++
std::optional<std::int32_t> get_node_level(Node& n);
```
- Returns an optional int with the value of the attribute level of node n

&nbsp;
  
```c++
std::optional<std::int32_t> get_parent_id(const Node& n);
```
- Returns an optional int with the value of the attribute parent of node n

&nbsp;

  
```c++
std::optional<Node> get_parent_node(const Node& n);
```
- Returns an optional Node with the value of the attribute parent of node n
If node does not exist
If parent attribute does no exist

&nbsp;
  
```c++
std::string get_node_type(Node& n);
```
- Returns a string the node’s type

&nbsp;

```c++
std::vector<Edge> get_edges_by_type(const std::string& type);
```

&nbsp;
```c++
std::vector<Edge> get_edges_by_type(const Node& node, const std::string& type);
```

&nbsp;
```c++
std::vector<Edge> get_edges_to_id(int id);
```

&nbsp;
```c++
std::optional<std::map<EdgeKey, Edge>> get_edges(int id);
```

&nbsp;

## Auxiliary sub-APIs

### RT sub-API

These methods provide specialized access to RT edges.

The api has to be instantiated with: `auto rt = G->get_rt_api()`;

&nbsp;
```c++
void insert_or_assign_edge_RT(Node& n, int to, const std::vector<float>& trans, const std::vector<float>& rot_euler);
```
 - Inserts or replaces an edge of type RT going from node n to node with id to. The translation vector is passed as a vector of floats. The three euler angles are passed as a vector of floats.

&nbsp;
```c++
void insert_or_assign_edge_RT(Node& n, int to, std::vector<float>&& trans, std::vector<float>&& rot_euler);
```

&nbsp;
#### Overloaded method using move semantics.
```c++
Edge get_edge_RT(const Node &n, int to);
```
- Returns an edge of type RT going from node n to node with id to
```c++
RTMat get_edge_RT_as_RTMat(const Edge &edge);
```
- Returns the rotation and translation attributes of edge converted to an RTMat.

&nbsp;

  
```c++
RTMat get_edge_RT_as_RTMat(Edge &&edge);
```
- Overloaded method with move semantics.

&nbsp;
  

### IO sub-API

These methods provide serialization to and from a disk file and display printing services.

The api has to be instantiated with: `auto io = G->get_io_api();`

  
```c++
void print();
```

&nbsp;
```c++
void print_edge(const Edge &edge) ;
```

&nbsp;
```c++
void print_node(const Node &node);
```

&nbsp;
```c++
void print_node(int id);
```

&nbsp;
```c++
void print_RT(std::int32_t root) const;
```

&nbsp;
```c++
void write_to_json_file(const std::string &file) const;
```

&nbsp;
```c++
void read_from_json_file(const std::string &file) const;
```

&nbsp;
  

### Innermodel sub-API

These methods compute transformation matrices between distant nodes in the so-called RT- tree that is embedded in G. The transformation matrix responds to the following question: how is a point defined in A’s reference frame, seen from B’s reference frame? It also provides backwards compatibility with RobComp’s InnerModel system.

The api has to be instantiated with: `auto inner = G->get_inner_api();`

&nbsp;
```c++
void transformS(std::string to, std::string from);
```

&nbsp;
```c++
void transformS(const std::string &to, const QVec &point, const std::string &from);
```

&nbsp;
```c++
void transform(QString to, QString from);
```

&nbsp;
```c++
void transform(QString to, const QVec &point, QString from);
```

&nbsp;

## CRDT- API

```c++
void reset()
```

&nbsp;
```c++
void update_maps_node_delete(int id, const Node& n);
```

&nbsp;
```c++
void update_maps_node_insert(int id, const Node& n);
```

&nbsp;
```c++
void update_maps_edge_delete(int from, int to, const std::string& key);
```

&nbsp;

## Node struct

G is defined at the middleware level as a struct with four fields and two maps. The first map holds a dictionary of attributes and the second one a dictionary of edges. Names of attributes (keys) and their types must belong to a predefined set, listed in XXX.  
  
```c++
struct Node {
	string type;
	string name;
	long id;
	long agent_id;
	map<string, Attrib> attrs;
	map<EdgeKey, Edge> fano;
};
```

&nbsp;
 
 Edges are indexed with a compound key:
```c++
struct EdgeKey {
	long to;
	string type;
};
```
formed by the id of the destination node and the type of the node. Possible types must belong to a predefined set listed in XXX. Each edge has three fields and a map.
```c++
struct Edge {
	long to; //key1
	string type; //key2
	long from;
	map<string, Attrib> attrs;
};
```

The destination node and the type, which both form part of the key, the id of the current node, to facilitate edge management, and a map of attributes.

Attributes are defined as a struct with a type and a value of type Val (a timestamp with the last modification will be added shortly)
```c++
struct _Attrib {
	long type;
	Val value;
};
```

Val is defined as a union of types, including string, int, float, vector of float and boolean (vector of byte will be added shortly).

```c++
union Val switch(long) {
	case 0:
		string str;
	case 1:
		long dec;
	case 2:
		float fl;
	case 3:
		sequence<float> float_vec;
	case 4:
		boolean bl;
	case 5:  
		sequence<octet> byte_vec;
};
```
 

These structures are compiled into C++ code that is included in the agent, forming the deeper layer of G. On top of it, another layer called CRDT is added to provide eventual consistency while agents communicate using asynchronous updates.



<!--stackedit_data:
eyJoaXN0b3J5IjpbODMyNzA2MzkzLC05MjQ5MjgzODQsLTExNz
IzNTEwOTIsLTIwNzQzNzUyNzIsMTExODkxNDczNiw5OTAxODcz
NTEsMTgzMDA4MDQ0MCwtMTU3NDE1NjQxMCw2OTU4NTA3Ml19
-->
