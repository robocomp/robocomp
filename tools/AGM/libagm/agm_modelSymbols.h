#pragma once

#include <stdint.h>

#include <string>
#include <vector>
#include <map>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

class AGMModel;
class AGMModelConverter;
class AGMModelEdge;
class AGMModelSymbol;


/*!
 * @brief Nodes --symbols-- of AGMModel graphs
 *
 * @ingroup CPPAPI
 *
 *
 */
class AGMModelSymbol
{
friend class AGMModel;
public:
	/*!
		* @brief Iterator class for accessing the edges of a particular symbol.
		*
		*
		*
		*/
	class iterator
	{
	public:
		/// Default constructor
		iterator() { modelRef = NULL; symRef = NULL; index = -1; }
		iterator(const AGMModelSymbol::iterator &o) { index=o.index; modelRef=o.modelRef; symRef=o.symRef; }
		iterator(AGMModel *m, AGMModelSymbol *s);
		/// Copy constructor
		iterator(iterator &iter);
		/// Access to the begin of the list.
		static iterator begin(AGMModel *m, AGMModelSymbol *s);
		/// Access to an unaccessible element of the list.
		static iterator end(AGMModel *m, AGMModelSymbol *s);
		/// Comparison operator
		bool operator==(const iterator &rhs);
		/// Not-equal operator
		bool operator!=(const iterator &rhs);
		/// Increment
		iterator operator++();
		/// Parametrized increment
		iterator operator++(int32_t times);
		/// Get referenced edge.
		AGMModelEdge & operator*();
		/// Get referenced edge.
		AGMModelEdge & operator->();
// 	private:
		int32_t index;
		AGMModel *modelRef;
		AGMModelSymbol *symRef;
	};

private:
	AGMModelSymbol(AGMModel *model, std::string typ, int32_t id=-1);
	AGMModelSymbol(AGMModel *model, int32_t identifier, std::string typ);
	AGMModelSymbol(AGMModel *model, int32_t identifier, std::string typ, std::map<std::string, std::string> atr);
	AGMModelSymbol(boost::shared_ptr<AGMModel> model, std::string typ, int32_t id=-1);
	AGMModelSymbol(boost::shared_ptr<AGMModel> model, int32_t identifier, std::string typ);
	AGMModelSymbol(boost::shared_ptr<AGMModel> model, int32_t identifier, std::string typ, std::map<std::string, std::string> atr);
public:
	~AGMModelSymbol();
private:
	void init(AGMModel *model, std::string typ, int32_t id=-1);
	void init(AGMModel *model, int32_t identifier, std::string typ);
	void init(AGMModel *model, int32_t identifier, std::string typ, std::map<std::string, std::string> atr);
public:
	typedef boost::shared_ptr<AGMModelSymbol> SPtr;
	std::string toString(bool verbose = false) const;
	std::string typeString() const;
	std::string symboltype() const { return symbolType; }

	void setType(std::string t);
	void setIdentifier(int32_t t);
	void setAttribute(std::string a, std::string v);
	std::string getAttribute(const std::string &a, bool debug = false) const;

	AGMModelSymbol::iterator edgesBegin(AGMModel *m)
	{
		return AGMModelSymbol::iterator::begin(m, this);
	}
	AGMModelSymbol::iterator edgesBegin(boost::shared_ptr<AGMModel> m)
	{
		return edgesBegin(m.get());
	}
	AGMModelSymbol::iterator edgesEnd(AGMModel *m)
	{
		return AGMModelSymbol::iterator::end(m, this);
	}
	AGMModelSymbol::iterator edgesEnd(boost::shared_ptr<AGMModel> m)
	{
		return edgesEnd(m.get());
	}

	AGMModelSymbol::iterator begin()
	{
		return edgesBegin(modelRef);
	}
	AGMModelSymbol::iterator end()
	{
		return AGMModelSymbol::iterator::end(modelRef, this);
	}


	bool operator==(const AGMModelSymbol &p) const;

	AGMModel *modelRef;
	std::string symbolType;
	int32_t identifier;
	std::map<std::string, std::string> attributes;

};



