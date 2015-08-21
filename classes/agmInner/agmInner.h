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
#include <agm.h>
#include <qt4/QtCore/QHash>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/QString>
#include <qt4/QtCore/QStringList>
#include <iostream>
#include <string>


using namespace std;


class AgmInner
{
public:
	AgmInner();
	~AgmInner();
	//const AGMModel::SPtr &src
	void setWorld(AGMModel::SPtr model);
	AGMModel::SPtr getWorld();
	int findName(QString n);
	int findName(const AGMModel::SPtr &m, QString n);		
	InnerModel* extractInnerModel(QString imNodeName="world");
	AGMModel::SPtr extractAGM();
	void recorrer(InnerModel* imNew, int& symbolID);
	void edgeToInnerModel(AGMModelEdge edge, InnerModel* imNew);
	QList< int > getLinkedID(int symbolID, string linkType);
	void checkLoop(int& symbolID, QList< int >& visited, string linkType, bool& loop);
	void updateAgmWithInnerModel(InnerModel* im);
	void updateAgmWithInnerModelAndPublish(InnerModel* im, AGMAgentTopicPrx &agmagenttopic_proxy);
	void insertSymbolToInnerModelNode(InnerModel* imNew, InnerModelNode *parentNode, AGMModelSymbol::SPtr s, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0);
	AGMModel::SPtr remove_ImOriginal(string agmFilePath, string imFilePath);
	
	//includes methods
	void include_im(QHash<QString, int32_t>  match, InnerModel *im);
	void innerToAGM(InnerModelNode* node, int &symbolID, QList<QString>  lNode);
	map< string, string > ImNodeToSymbol(InnerModelNode* node);
	void  remove_Im( InnerModel*imTmp);
	
	void updateImNodeFromEdge(AGMModelEdge edge, InnerModel *innerModel);



private:
    AGMModel::SPtr worldModel;
    InnerModel *innerModel;
	
};

#endif // AGMINNER_H
