#pragma once

#include <agm_config.h>
#include <agm_model.h>
#include <agm_modelEdge.h>
#include <agm_modelSymbols.h>

#if ROBOCOMP_SUPPORT == 1
#include <AGMWorldModel.h>
#endif

/*!
 * @brief Utility class used to convert between different graph representation.
 *
 * 
 * 
 */
class AGMModelConverter
{
public:

#if ROBOCOMP_SUPPORT == 1
	///World
	/// Converts an AGM world model from an AGMModel container to a RoboCompAGMWorldModel::World container.
	static void fromInternalToIce(const AGMModel::SPtr &world, RoboCompAGMWorldModel::World &dst);
	/// Converts an AGM world model from a RoboCompAGMWorldModel::World container to an AGMModel container.
	static void fromIceToInternal(const RoboCompAGMWorldModel::World &world, AGMModel::SPtr &dst);

	///Nodes
	/// Converts an AGM symbol model from an AGMModelSymbol container (given by a shared pointer) to a RoboCompAGMWorldModel::Node container.
	static void fromInternalToIce(const AGMModelSymbol::SPtr &node, RoboCompAGMWorldModel::Node &dst);
	/// Converts an AGM symbol model from an AGMModelSymbol container (given by a regular pointer) to a RoboCompAGMWorldModel::Node container.
	static void fromInternalToIce(const AGMModelSymbol *node, RoboCompAGMWorldModel::Node &dst);
	/// Converts an AGM symbol model from a RoboCompAGMWorldModel::Node container to an AGMModelSymbol container.
	static void fromIceToInternal(const RoboCompAGMWorldModel::Node &node, AGMModelSymbol::SPtr &dst);
	
	///Edges
	/// Converts an AGM edge model from an AGMModelEdge container (given by a shared pointer) to a RoboCompAGMWorldModel::Edge container.
// 	static void fromInternalToIce(const AGMModelEdge::SPtr &edge, RoboCompAGMWorldModel::Edge &dst);
	/// Converts an AGM edge model from an AGMModelEdge container (given by a regular pointer) to a RoboCompAGMWorldModel::Edge container.
	static void fromInternalToIce(const AGMModelEdge *edge, RoboCompAGMWorldModel::Edge &dst);
	/// Converts an AGM symbol model from a RoboCompAGMWorldModel::Edge container to an AGMModelEdge container.
	static void fromIceToInternal(const RoboCompAGMWorldModel::Edge &edge, AGMModelEdge &dst);


	/// Updates an AGM world model (an AGMModel object) using an updated node (given a RoboCompAGMWorldModel::Node instance).
	static bool includeIceModificationInInternalModel(const RoboCompAGMWorldModel::Node &node, AGMModel::SPtr &world);
	static bool includeIceModificationInInternalModel(const std::vector<RoboCompAGMWorldModel::Node> &nodes, AGMModel::SPtr &world);

	/// Updates an AGM world model (an AGMModel object) using an updated edge (given a RoboCompAGMWorldModel::Edge instance).
	static bool includeIceModificationInInternalModel(const RoboCompAGMWorldModel::Edge &edge, AGMModel::SPtr &world);
	static bool includeIceModificationInInternalModel(const std::vector<RoboCompAGMWorldModel::Edge> &edges, AGMModel::SPtr &world);
#endif

	/// Creates an AGM world model (AGMModel object) given the path to an existing XML description.
	static void fromXMLToInternal(const std::string path, AGMModel::SPtr &dst);


};


