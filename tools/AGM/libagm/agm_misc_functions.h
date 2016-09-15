#pragma once

#include <agm_config.h>
#include <agm_model.h>
#include <agm_modelException.h>

#if ROBOCOMP_SUPPORT == 1
#include <AGMExecutive.h>
#include <AGMWorldModel.h>
#include <AGMCommonBehavior.h>
using namespace RoboCompAGMExecutive;
#endif

/*! Converts an STD string to float. */
float str2float(const std::string &s, bool debug = false);
/*! Converts an STD string to integer. */
int32_t str2int(const std::string &s);


/*! Converts a float to an STD string. */
std::string float2str(const float &f);
/*! Converts an integer to an STD string. */
std::string int2str(const int32_t &i);




/*!
 * \class AGMMisc
 * @ingroup CPPAPI
 * @brief Class containing several useful functions.
 *
 * Class containing several useful functions.
 * 
 */
class AGMMisc
{
public:
#if ROBOCOMP_SUPPORT == 1
	/*! Proposes a new world model (<em>newModel</em>) using the proxy <em>AGMExecutive</em> provinding <em>sender</em> as additional information that might be interesting for debugging purposes. */
	static void publishModification(AGMModel::SPtr &newModel, AGMExecutivePrx &agmexecutive, std::string sender="unspecified");
	/*! Modifies a node (<em>symbol</em>) using the proxy <em>AGMExecutive</em>. */
	static void publishNodeUpdate(AGMModelSymbol::SPtr &symbol, AGMExecutivePrx &agmexecutive);
	/*! Modifies several nodes (<em>symbol</em>) using the proxy <em>AGMExecutive</em>. */
	static void publishNodesUpdate(std::vector<AGMModelSymbol::SPtr> symbols, AGMExecutivePrx &agmexecutive);
	/*! Modifies an edge (<em>edge</em>) using the proxy <em>AGMExecutive</em>. */
	static void publishEdgeUpdate(AGMModelEdge &edge, AGMExecutivePrx &agmexecutive);
	/*! Modifies several edges (<em>edge</em>) using the proxy <em>AGMExecutive</em>. */
	static void publishEdgesUpdate(std::vector<AGMModelEdge> edges, AGMExecutivePrx &agmexecutive);
#endif
	static inline float str2float(const std::string &s, bool debug = false);
	static inline int32_t str2int(const std::string &s);
	static inline std::string float2str(const float &f);
	static inline std::string int2str(const int32_t &i);
};

