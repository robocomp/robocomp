/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



// THIS IS AN AGENT




#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#ifndef Q_MOC_RUN
 #include <agm.h>
#endif

#include <QHash>
#include <QList>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

// 	bool reloadConfigAgent();
// 	bool activateAgent(const ParameterMap &prs);
// 	bool setAgentParameters(const ParameterMap &prs);
// 	ParameterMap getAgentParameters();
// 	void killAgent();
// 	int uptimeAgent();
// 	bool deactivateAgent();
// 	StateStruct getAgentState();
// 	void structuralChange(const RoboCompAGMWorldModel::World &modification);
// 	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
// 	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
// 	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
// 	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification);


public slots:
	void compute();


public:
	std::vector<std::pair<InnerModel *, QString> > innerModelInfoVector;
private:
	bool gotModel;
	std::string action;
// 	ParameterMap params;
	AGMModel::SPtr worldModel;
	osgGA::TrackballManipulator *manipulator;
	OsgView *osgView;
	InnerModel *innerModelVacio;
	InnerModelViewer *innerViewer;

	bool active;
// 	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
// 	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);


	void innerToAGM(InnerModelNode* node, int &symbolID, QList<QString>  lNode);
	//void include_im(QHash< QString, ::int32_t > match);
// 	void include_im(QHash<QString, int32_t>  match, InnerModel *im);


	int findName(QString n);
	AGMModelSymbol::SPtr ImNodeToSymbol(InnerModelNode* node);
// 	InnerModel* extractInnerModel(QString imNodeName);
// 	void recorrer(InnerModel* imNew, int& symbolID);
// 	void edgeToInnerModel(AGMModelEdge edge, InnerModel* imNew);

// 	void checkLoop(int& symbolID, QList< int >& visited, string linkType, bool& loop);

};

#endif

