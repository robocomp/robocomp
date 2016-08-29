#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>


#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <agm_modelException.h>

#if ROBOCOMP_SUPPORT == 1
#include <AGMExecutive.h>
#include <AGMWorldModel.h>
#include <AGMCommonBehavior.h>
#endif

#include <agm_modelSymbols.h>


int32_t str2int(const std::string &s);

class AGMModelEdge;
class AGMModelConverter;
class AGMModelSymbol;


/*!
 * \class AGMModel
 * @ingroup CPPAPI
 * @brief Graph representation for robots' models
 *
 * More detailed information to come...
 *
 */
class AGMModel
{
friend class AGMModelConverter;
friend class AGMModelSymbol;
public:
	/*!
	 *  @brief Iterator class for accessing the symbols in an AGMModel object.
	 *
	 */
	class iterator
	{
	public:
		/// Constructor
		iterator();
		/// Copy constructor
		iterator(iterator &iter);
		iterator(const iterator &iter);
		/// Access to the begin of the list.
		static iterator begin(AGMModel *m);
		static iterator begin(boost::shared_ptr<AGMModel> m);
		/// Access to an unaccessible element of the list.
		static iterator end(AGMModel *m);
		static iterator end(boost::shared_ptr<AGMModel> m);
		/// Comparison operator
		bool operator==(const iterator &rhs);
		/// Not-equal operator
		bool operator!=(const iterator &rhs);
		/// Increment
		iterator operator++();
		iterator operator++(int32_t);
		/// Returns the referenced symbol.
		AGMModelSymbol::SPtr operator*();
		/// Returns the referenced symbol.
		AGMModelSymbol::SPtr operator->();
// 	private:
		int32_t index;
		AGMModel *modelRef;
	};


	//
	// Constructors and related
	//

	//! Shared pointer type to point to AGMModel objects.
	typedef boost::shared_ptr<AGMModel> SPtr;
	//! Default constructor.
	AGMModel();
	//! Copy constructor using a shared pointer.
	AGMModel(const AGMModel::SPtr &src);
	//! Copy constructor using a reference.
	AGMModel(const AGMModel &src);
	//! File constructor.
	AGMModel(const std::string xmlFilePath);
	//! Destructor.
	~AGMModel();
	//! Assignment operator.
	AGMModel& operator=(const AGMModel &src);

	iterator begin()
	{
		return AGMModel::iterator::begin(this);
	}

	iterator end()
	{
		return AGMModel::iterator::end(this);
	}

	/// Empties the model.
	void clear();

	/*! Removes a particular symbol of the model given its identifier. It must be noted that by removing a symbol <strong>we will also delete all edges related to such symbol</strong>.
	 * \attention It must be noted that by removing a symbol <strong>we will also delete all edges related to such symbol</strong>.
	 */
	void removeSymbol(int32_t id);


	/*! Removes a particular symbol of the model given its identifier. It must be noted that by removing a symbol <strong>we will also delete all edges related to such symbol</strong>.
	 * \attention It must be noted that by removing a symbol <strong>we will also delete all edges related to such symbol</strong>.
	 */
	void removeSymbol(AGMModelSymbol::SPtr s)
	{
		removeSymbol(s->identifier);
	}


	/// Replaces the first of the indentifiers with the second one in all edges of the model. It is very useful when creating a new symbol that is meant to substitute another existing symbol.
	int32_t replaceIdentifierInEdges(int32_t existing, int32_t replacement);


	/// Returns a shared pointer to a new symbol of type <em>symbolType</em>, with identifier <em>id</em>. If omitted, <em>id</em> is set to <em>-1</em>, which makes the symbol to be automatically assigned the first available identifier.
	AGMModelSymbol::SPtr newSymbol(std::string symbolType, int32_t id=-1);


	/// Returns a shared pointer to a new symbol of type <em>symbolType</em>, with identifier <em>id</em>. If <em>id</em> is set to <em>-1</em>, which makes the symbol to be automatically assigned the first available identifier.
	AGMModelSymbol::SPtr newSymbol(int32_t identifier, std::string symbolType);


	/// Returns a shared pointer to a new symbol of type <em>symbolType</em>. Its behavior is the one of the previous method, but it also sets for the new symbol the map of attributes provided as <em>attrs</em>.
	AGMModelSymbol::SPtr newSymbol(int32_t identifier, std::string symbolType, std::map<std::string, std::string> attrs);

	AGMModelSymbol::SPtr newSymbol(std::string symbolType, std::map<std::string, std::string> attrs);


	/// Removes all edges related to the symbol with identifier <em>id</em>.
	bool removeEdgesRelatedToSymbol(int32_t id);


	/// Removes all edges related to an unexisting symbol. Dangling edges only exist if users remove symbols using the variable <em>symbols</em> directly, which is disencouraged. Generally it's a better idea to remove symbols using the method <em>removeSymbol</em>.
	bool removeDanglingEdges();

	void save(std::string xmlFilePath);


private:

	/// Inserts a given symbol in the model given a shared pointer to it.
	int32_t insertSymbol(AGMModelSymbol::SPtr s);


	/// Inserts a given symbol in the model given a regular pointer to it.
	int32_t insertSymbol(AGMModelSymbol *s) { return insertSymbol(AGMModelSymbol::SPtr(s)); }
public:


	/// Name of the model. Only used for debugging purposes.
	std::string name;


	/// Returns a shared pointer to the symbol of identifier <em>identifier</em>
	AGMModelSymbol::SPtr getSymbol(int32_t identifier) const;


	/// Returns the number of edges in the model.
	int32_t numberOfEdges() const;


	/// Returns the number of symbols in the model.
	int32_t numberOfSymbols() const;


	/*! \brief Returns the index of a given symbol '<em>sym</em>' (shared pointer) within the model. If not found, returns <em>-1</em>.
	 *
	 *  Optionally, the user can provide an offset within the symbol vector (zero by default).
	 */
	int32_t indexOfSymbol(const AGMModelSymbol::SPtr &sym, int32_t from=0) const;


	/*! \brief Returns the index of a given symbol '<em>sym</em>' (reference) within the model. If not found, returns <em>-1</em>.
	 *
	 *  Optionally, the user can provide an offset within the symbol vector (zero by default).
	 */
	int32_t indexOfFirstSymbolByValues(const AGMModelSymbol &sym, int32_t from=0) const;


	/*! \brief Returns the index of the first symbol of type '<em>symbolType</em>' within the model. If not found, returns <em>-1</em>.
	 *
	 *  Optionally, the user can provide an offset within the symbol vector (zero by default).
	 */
	int32_t indexOfFirstSymbolByType(const std::string &symbolType, int32_t from=0) const;


	/// Returns a reference to the symbol vector. Use with care.
	std::vector<AGMModelSymbol::SPtr> getSymbols() const;

#if ROBOCOMP_SUPPORT == 1
	/// Returns a map of smart pointers to a list of symbools
	template <typename T>
	std::map<std::string, AGMModelSymbol::SPtr>getSymbolsMap(::RoboCompAGMCommonBehavior::ParameterMap params, T t)
	{
		std::map<std::string, AGMModelSymbol::SPtr> ret;
		ret[t] = getSymbolByIdentifier(str2int(params[t].value));
		return ret;
	}
	template<typename T, typename... Args>
	std::map<std::string, AGMModelSymbol::SPtr>getSymbolsMap(::RoboCompAGMCommonBehavior::ParameterMap params, T t, Args... args)
	{
		std::map<std::string, AGMModelSymbol::SPtr> ret, recurs;
		ret[t] = getSymbolByIdentifier(str2int(params[t].value));
		recurs = getSymbolsMap(params, args...);
		ret.insert(recurs.begin(), recurs.end());
		return ret;
	}

	std::map<std::string, AGMModelSymbol::SPtr>getSymbolsMap(::RoboCompAGMCommonBehavior::ParameterMap params);

// 	worldModel->getSymbolsMap(params, "person", "objectr", "conth");
#endif

	/// Returns a reference to the edge vector. Use with care. <strong>DEPRECATED</STRONG>
// 	std::vector<AGMModelEdge> getEdges() const;


	/*! \brief Returns a reference to the symbol located in position <em>'index'</em> within the symbol vector.
	 *
	 * \attention Note that <em>'index'</em> refers to the symbol's index within the symbol vector, not to its identifier.
	 */
	AGMModelSymbol::SPtr &symbol(uint32_t index);


	/// \brief Returns a reference to the edge located in position <em>'index'</em> within the edge vector.
	AGMModelEdge &edge(uint32_t index);


	/*! \brief Returns the identifier of the symbol with <strong><em>"name"</em></strong> <em>'name'</em>, where a symbol's name is considered to be the concatenation of its type, the underscore character ('_') and its identifier. If not found returns <em>-1</em>.
	 *
	 * This method is handy when interacting with the AGM planner.
	 *
	 * \attention A symbol's name is considered to be the concatenation of its type, the underscore character ('_') and its identifier.
	 */
	int32_t getIdentifierByName(std::string name) const;


	/// Returns the identifier of the symbol with <strong><em>"symbolType"</em></strong> type. If not found returns <em>-1</em>.
	int32_t getIdentifierByType(std::string symbolType, int32_t i=0) const;


	/*! \brief <strong>DEPRECATED</STRONG>: DON'T USE THIS METHOD
	 *
	 * \deprecated This method is kept only to avoid breaking old code but shouldn't be used in new code. It will soon be removed from the API. Please, don't use this method anymore.
	 *
	 */
	int32_t getLinkedID(    int32_t id, std::string linkname, int32_t i=0) const;


	AGMModelSymbol::SPtr getParentByLink(int32_t id, std::string linkname, int32_t i=0) const;


	/// Returns the index of the symbol with identifier '<em>targetId</em>, <em>-1</em> if the symbol is not found.
	int32_t getIndexByIdentifier(int32_t targetId) const;


	/*! \brief Returns a shared pointer to the symbol with identifier '<em>targetId</em>'.
	 *
	 * \throws AGMException If no symbol with identifier 'targetId' is found.
	 *
	 */
	AGMModelSymbol::SPtr getSymbolByIdentifier(int32_t targetId) const;


	/*! \brief Returns a shared pointer to the symbol with name '<em>name</em>', where a symbol's name is considered to be the concatenation of its type, the underscore character ('_') and its identifier.
	 *
	 * \throws AGMException If no symbol with identifier 'targetId' is found.
	 *
	 */
	AGMModelSymbol::SPtr getSymbolByName(const std::string &name) const;


	/*! \brief Includes a new edge in the model given the identifiers of two symbols with an optional attribute map.  Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	void addEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName, std::map<std::string, std::string> atr=std::map<std::string, std::string>());


	/*! \brief Includes a new edge from the symbol 'a' to the symbol 'b', with an optional attribute map.  Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	void addEdge(AGMModelSymbol::SPtr a, AGMModelSymbol::SPtr b, const std::string &edgeName, std::map<std::string, std::string> atr=std::map<std::string, std::string>())
	{
		addEdgeByIdentifiers(a->identifier, b->identifier, edgeName, atr);
	}


	/*! \brief Removes a new edge in the model given the identifiers of two symbols and the label. Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	void removeEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName);


	/*! \brief Includes a new edge from the symbol 'a' to the symbol 'b' with label edgeName, with an optional attribute map.  Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	void removeEdge(AGMModelSymbol::SPtr a, AGMModelSymbol::SPtr b, const std::string &edgeName)
	{
		removeEdgeByIdentifiers(a->identifier, b->identifier, edgeName);
	}


	/*! \brief Renames an edge in the model given the identifiers of two symbols, the previous and new label. Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	bool renameEdgeByIdentifiers(int32_t a, int32_t b, const std::string &was, const std::string &will);


	/*! \brief Renames an edge in the model given the identifiers of two symbols, the previous and new label. Returns True on success.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	bool renameEdge(AGMModelSymbol::SPtr a, AGMModelSymbol::SPtr b, const std::string &was, const std::string &will)
	{
		return renameEdgeByIdentifiers(a->identifier, b->identifier, was, will);
	}

	/*! \brief get edge by indentifiers of two symbols and label.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	AGMModelEdge &getEdgeByIdentifiers(int32_t a, int32_t b, const std::string &edgeName);

	/*! \brief get edge by providing two symbols and a label.
	 *
	 * \throws AGMException Nodes a and b must exist
	 *
	 */
	AGMModelEdge &getEdge(AGMModelSymbol::SPtr a, AGMModelSymbol::SPtr b, const std::string &edgeName);



	/*! \brief Automatically updates the next available identifier as the smaller identifier that is bigger than any of the existing ones.
	 *
	 */
	void resetLastId();


	/*! \brief  Set the next available identifier as <em>'i'</em>. When possible, use AGMModel::resetLastId() instead.
	 *
	 */
	void setLastId(int32_t i) { lastId = i; }


	/*! \brief Returns the next available identifier and makes the returned value unavailable.
	 *
	 */
	int32_t getNewId();


	/*! \brief Returns a string containing the PDDL description of a planning problem given the current model and a target model.
	 *
	 * The AGMModel instance which is used to generate the problem is used as initial world and '<em>target</em>' represents the target world.
	 *
	 * <em>unknowns</em> represents the maximum number of new symbols that the PDDL planner will use.
	 * <em>domainName</em> is the name of the PDDL domain which is supposed to be use for the generated problem.
	 * <em>problemName</em> is the name that the generated PDDL problem will be given.
	 */
	std::string generatePDDLProblem(const AGMModel::SPtr &target, int32_t unknowns, const std::string domainName, const std::string problemName="problemName") const;


	/// Vector of the symbols that the model holds.
	std::vector<AGMModelSymbol::SPtr> symbols;
	int32_t size() { return symbols.size(); }
	/// Vector of the edges that the model holds.
	std::vector<AGMModelEdge> edges;
	/// Version ID of the model
	int32_t version;
private:

	/// Overwrites the symbols attribute with the vector provided.
	void setSymbols(std::vector<AGMModelSymbol::SPtr> s);

	/// Overwrites the edges attribute with the vector provided.
	void setEdges(std::vector<AGMModelEdge> e);

	/// Overwrites the whole objects with the attributes in the model provied.
	void setFrom(const AGMModel &src);

	/// This variable holds the next available symbol identifier.
	int32_t lastId;

};


