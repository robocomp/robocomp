#ifndef ROBOCOMPAGMEXECUTIVE_ICE
#define ROBOCOMPAGMEXECUTIVE_ICE
#include <AGMWorldModel.ice>
#include <Planning.ice>
module RoboCompAGMExecutive
{
	exception Locked{ };
	exception OldModel{ };
	exception InvalidChange{ };
	interface AGMExecutive
	{
		void activate ();
		void deactivate ();
		void structuralChangeProposal (RoboCompAGMWorldModel::World w, string sender, string log) throws Locked,OldModel,InvalidChange;
		void symbolUpdate (RoboCompAGMWorldModel::Node n);
		void symbolsUpdate (RoboCompAGMWorldModel::NodeSequence ns);
		void edgeUpdate (RoboCompAGMWorldModel::Edge e);
		void edgesUpdate (RoboCompAGMWorldModel::EdgeSequence es);
		void addSelfEdge (int nodeid, string edgeType, RoboCompAGMWorldModel::StringDictionary attributes);
		void delSelfEdge (int nodeid, string edgeType);
		void setMission (string path);
		RoboCompAGMWorldModel::World getModel ();
		RoboCompAGMWorldModel::Node getNode (int identifier);
		RoboCompAGMWorldModel::Edge getEdge (int srcIdentifier, int dstIdentifier, string label);
		void getData (out RoboCompAGMWorldModel::World world, out string target, out RoboCompPlanning::Plan plan);
		void broadcastModel ();
		void broadcastPlan ();
	};
};

#endif
