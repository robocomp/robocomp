#ifndef ROBOCOMPAGMEXECUTIVE2_ICE
#define ROBOCOMPAGMEXECUTIVE2_ICE

#include <AGMWorldModel2.ice>
#include <Planning.ice>

module RoboCompAGMExecutive
{
	enum ProposalError { NoError, Locked, OldModel, InvalidChange };

	interface AGMExecutive
	{
		// Activation and deactivation
		void activate();
		void deactivate();

		// For agents
		ProposalError modificationProposal(RoboCompAGMWorldModel::World w, string sender, string log);

		// For setting the mission
		void setMission(string path);

		// Access to the graph structure
		RoboCompAGMWorldModel::World getModel();
		RoboCompAGMWorldModel::Node getNode(int identifier);
		RoboCompAGMWorldModel::Edge getEdge(int srcIdentifier, int dstIdentifier, string label);

		// For visualization purposes
		void getData(out RoboCompAGMWorldModel::World world, out RoboCompAGMWorldModel::World target, out RoboCompPlanning::Plan plan);
		void broadcastModel();
		void broadcastPlan();
	};

	interface AGMExecutiveVisualizationTopic
	{
		void update(RoboCompAGMWorldModel::World world, RoboCompAGMWorldModel::World target, RoboCompPlanning::Plan plan);
		void successFulChange(RoboCompPlanning::ActionSequence actions);
		void aimedChange(RoboCompPlanning::Action action);
	};

	interface AGMExecutiveTopic
	{
		void structuralChange(RoboCompAGMWorldModel::World w);

		void symbolUpdated(RoboCompAGMWorldModel::Node n);
		void symbolUpdated(RoboCompAGMWorldModel::NodeSequence ns);

		void edgeUpdated(RoboCompAGMWorldModel::Edge e);
		void edgesUpdated(RoboCompAGMWorldModel::EdgeSequence es);
	};
};
  
#endif