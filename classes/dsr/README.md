# DSR (Deep State Representation)
  * [Description](#Description)
  * [DSR-API (aka G-API)](#dsr-api--aka-g-api-)
  * [CORE](#core)
  * [Auxiliary sub-APIs](#auxiliary-sub-apis)
    + [RT sub-API](#rt-sub-api)
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

![dsr-graph](https://user-images.githubusercontent.com/5784096/89400000-88576200-d713-11ea-8ac2-d2980568c2dc.png)

<sup>*The illustration shows a possible instance of the CORTEX architecture. The central part of the ring contains the **DSR graph** that is shared by all agents, from whom a reference implementation is presented here. Coloured boxes represent agents providing different functionalities to the whole. The purple box is an agent that can connect to the real robot or to a realistic simulation of it, providing the basic infrastructure to explore prediction and anticipation capabilities*</sup>

Conceptually, the DSR represents a network of entities and relations among them. Relations can be unary or binary predicates, while the entities may have complex numeric properties such as pose transformation matrices that represent the kinematic relations of objects in the world and the robot’s parts. Mathematically, the DSR is internalized as a directed graph with attributed edges. As a hybrid representation that stores information at both geometric and symbolic levels, the nodes of the DSR store concepts that can be symbolic, geometric or a combination of both. Metric concepts describe numeric quantities of objects in the world, which can be structures such as a three-dimensional mesh, scalars such as the mass of a link, or lists such as revision dates. Edges represent relationships between nodes. Two nodes may have several kinds of relationships but only one of them can be geometric. The geometric relationship is expressed with a fixed label called *RT*. This label stores the transformation matrix (expressed as a rotation-translation) between them.

> This documentation describes the classes that allow the creation of agents to use this Deep State Representation.

## DSR-API (aka G-API)
G-API is the user-level access layer to G. It is composed by a set of core methods that access the underlying CRDT and RTPS APIs, and an extendable  set of auxiliary methods added to simplify the user coding tasks. 


The most important features of the G-API are:

-   It always works on a copy a node. The obtention of the copy is done by a core method using shared mutex technology. Once a copy of the node is returned, the user can edit it for as long as she wants. When it is ready, the node is reinserted in G using another core method. This feature makes the API thread-safe.
    
-   There are a group of methods, that include the word local in their name, created to change the attributes of a node. These methods do not reinsert the node back into G and the user is left with this responsibility.
    
-   DSRGraph has been created as a QObject to emit signals whenever a node is created, deleted or modified. Using this functionality, a set of graphic classes have been created to show in real-time the state of G. These classes can be connected at run-time to the signals. There is an abstract class from which all of them inherit that can be used to create more user-defined observers of G.
    
-   To create a new node, a unique identifier is needed. To guarantee this requirement, the node creation method places a RPC call to the special agent idserver, using standard RoboComp communication methods. Idserver returns a unique id that can be safely added to the new node.
    

-   G can be serialized to a JSON file from any agent but it is better to do it only from the idserver agent, to avoid the spreading of copies of the graph in different states.


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