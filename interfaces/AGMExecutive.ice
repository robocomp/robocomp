#ifndef ROBOCOMPAGMEXECUTIVE2_ICE
#define ROBOCOMPAGMEXECUTIVE2_ICE

#include <AGMWorldModel.ice>
#include <Planning.ice>

module RoboCompAGMExecutive
{
	exception Locked {};
	exception OldModel {};
	exception InvalidChange  {};

	interface AGMExecutive
	{
		// Activation and deactivation
		void activate();
		void deactivate();

		// Agents' API
		void structuralChangeProposal(RoboCompAGMWorldModel::World w, string sender, string log) throws Locked, OldModel, InvalidChange;
		void symbolUpdate(RoboCompAGMWorldModel::Node n);
		void symbolsUpdate(RoboCompAGMWorldModel::NodeSequence ns);
		void edgeUpdate(RoboCompAGMWorldModel::Edge e);
		void edgesUpdate(RoboCompAGMWorldModel::EdgeSequence es);

		// To setting the mission given a target path
		void setMission(string path);

		// Access to the graph structure
		RoboCompAGMWorldModel::World getModel();
		RoboCompAGMWorldModel::Node getNode(int identifier);
		RoboCompAGMWorldModel::Edge getEdge(int srcIdentifier, int dstIdentifier, string label);

		// For visualization purposes
		void getData(out RoboCompAGMWorldModel::World world, out string target, out RoboCompPlanning::Plan plan);
		void broadcastModel();
		void broadcastPlan();
	};

	interface AGMExecutiveVisualizationTopic
	{
		void update(RoboCompAGMWorldModel::World world, string mission, RoboCompPlanning::Plan plan);
		//void successFulChange(RoboCompPlanning::ActionSequence actions);
		//void aimedChange(RoboCompPlanning::Action action);
	};

	interface AGMExecutiveTopic
	{
		void structuralChange(RoboCompAGMWorldModel::World w);

		void symbolUpdated(RoboCompAGMWorldModel::Node modification);
		void symbolsUpdated(RoboCompAGMWorldModel::NodeSequence modifications);

		void edgeUpdated(RoboCompAGMWorldModel::Edge modification);
		void edgesUpdated(RoboCompAGMWorldModel::EdgeSequence modifications);
	};
};
  
#endif
