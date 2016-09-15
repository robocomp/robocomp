#ifndef WORLDMODELEDGE_H
#define WORLDMODELEDGE_H

#include <string>

#include <stdint.h>

#include "agm_model.h"

/*!
 * @brief Edges --relationships between symbols-- of AGMModel graphs
 *
 * @ingroup CPPAPI
 *
 *
 */
class AGMModelEdge
{
public:
	AGMModelEdge();
	~AGMModelEdge();
	AGMModelEdge(int32_t a, int32_t b, std::string linking_, std::map<std::string, std::string> atr=std::map<std::string, std::string>());
	AGMModelEdge(const AGMModelEdge &src);
	AGMModelEdge &operator=(const AGMModelEdge &src);
	AGMModelEdge *operator->() { return this; }

	inline std::string getLabel() const { return linking; }
	inline std::pair<int32_t, int32_t> getSymbolPair() const { return symbolPair; }

	void setLabel(std::string l);
	void setSymbolPair(std::pair<int32_t, int32_t> p);

	///
	void setAttribute(std::string a, std::string v);
	std::string getAttribute(std::string a);

	std::string toString(const AGMModel::SPtr &world, bool verbose = false) const;
	std::string toString(const AGMModel *world, bool verbose = false) const;

	void getStrings(const AGMModel::SPtr &world, std::string &label, std::string &a, std::string &b) const;
	void getStrings(const AGMModel *world, std::string &label, std::string &a, std::string &b) const;


// protected:
	std::string linking;
	std::pair<int32_t, int32_t> symbolPair;
	std::map<std::string, std::string> attributes;
private:
	void setFrom(const AGMModelEdge &src);
};

#endif
