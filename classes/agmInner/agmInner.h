/*
 * 
 *   Copyright (C) 2015 by YOUR NAME HERE
 *  
 *   This file is part of RoboComp
 *  
 *   RoboComp is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *  
 *   RoboComp is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *  
 *   You should have received a copy of the GNU General Public License
 *   along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
 

#ifndef AGMINNER_H
#define AGMINNER_H

#include <innermodel/innermodel.h>
#include <qmat/QMatAll>
#include <qt4/QtCore/QHash>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/QString>
#include <qt4/QtCore/QStringList>
#include <iostream>
#include <string>

#ifndef Q_MOC_RUN
	#include <agm.h>
#endif

using namespace std;

class AgmInner
{
public:
	AgmInner();
	~AgmInner();
	//const AGMModel::SPtr &src
	void setWorld(AGMModel::SPtr model);
	AGMModel::SPtr getWorld();
	
	//return the symbolID of the node that includes the innerModel name n.
	int findName(QString n);
	int findName(const AGMModel::SPtr &m, QString n);
	
	//return only the symbolic part of the graph
	AGMModel::SPtr extractAGM();
	
	//this three functions work together.
	InnerModel* extractInnerModel(QString imNodeName="world", bool ignoreMeshes=false);
	void recorrer(InnerModel* imNew, int& symbolID, bool ignoreMeshes);
	void edgeToInnerModel(AGMModelEdge edge, InnerModel* imNew, bool ignoreMeshes);
	//*********************************************
	
	//find a loop from a symbolID throught a fixed linkType
	void checkLoop(int& symbolID, QList< int >& visited, string linkType, bool& loop);
	
	//
	void updateAgmWithInnerModel(InnerModel* im);
	void updateAgmWithInnerModelAndPublish(InnerModel* im, AGMAgentTopicPrx &agmagenttopic_proxy);
	void insertSymbolToInnerModelNode(InnerModel* imNew, InnerModelNode *parentNode, AGMModelSymbol::SPtr s, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, bool ignoreMeshes=false);
	AGMModel::SPtr remove_ImOriginal(string agmFilePath, string imFilePath);
	
	//list of symbols from a symbolID throught a specific linkType
	QList< int > getLinkedID(int symbolID, string linkType);
	
	//update innermodel node from edges
	void updateImNodeFromEdge(AGMModelEdge edge, InnerModel *innerModel);
	void updateImNodeFromEdge(const RoboCompAGMWorldModel::Edge& edge, InnerModel *innerModel);
	
	//includes methods this methods work together
	//Insert innermodel in AGM graph matching nodes from innerModel to their correspondent symbols. 
	void include_im(QHash<QString, int32_t>  match, InnerModel *im);
	void innerToAGM(InnerModelNode* node, int &symbolID, QList<QString>  lNode);
	map< string, string > ImNodeToSymbol(InnerModelNode* node);
	
	//Dado un innerModel (impTmp) elimina de  AGM todos los symbolos que lo forman.
	void  remove_Im( InnerModel*imTmp);

private:
    AGMModel::SPtr worldModel;
    InnerModel *innerModel;
	
};

#endif // AGMINNER_H
