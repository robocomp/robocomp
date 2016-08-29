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

#pragma once
#ifndef AGMINNER_H
#define AGMINNER_H

#if ROBOCOMP_SUPPORT == 1

#include <innermodel/innermodel.h>
#include <qmat/QMatAll>
#include <qt4/QtCore/QHash>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/QString>
#include <qt4/QtCore/QStringList>
#include <iostream>
#include <string>


#include <AGMExecutive.h>

#ifndef Q_MOC_RUN
	#include <agm.h>
#endif

using namespace std;

/*!
 * \class AGMInner
 * @ingroup CPPAPI
 * @brief Class containg static methods related to extracting, and modifying InnerModel instances given an AGMModel and the other way around.
 *
 * More detailed information to come...
 *
 */
class AGMInner
{
private:
	//funcion recursiva auxiliar para la extracción de innermodel
	static void recorrer( AGMModel::SPtr &worldModel, InnerModel* imNew, int& symbolID, bool ignoreMeshes);

	//Función auxiliar para extractInnerModel. Extrae la información del arco y sigue con el proceso de creacción del árbol de innermodel
	static void edgeToInnerModel( AGMModel::SPtr &worldModel, AGMModelEdge edge, InnerModel* imNew, bool ignoreMeshes);

	//Convierte el símbolo s en un nodo de innerModel, y lo inserta como hijo de parentNode en imNew.
	static void insertSymbolToInnerModelNode(AGMModel::SPtr &worldModel, InnerModel* imNew, InnerModelNode *parentNode, AGMModelSymbol::SPtr s, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, bool ignoreMeshes=false);

	//Funcion auxiliar para insertar el arbol de innermodel en el grafo AGM
	static void recursiveInsertion(AGMModel::SPtr &worldModel, InnerModelNode* node, int &symbolID);

	//DEPRECATED
	//Función auxiliar para include_im.
	static void innerToAGM(AGMModel::SPtr &worldModel, InnerModelNode* node, int &symbolID, QList<QString>  lNode);

	//Extrae de un nodo de innermodel la información relevante a incorporar en el símbolo. En la forma de sus atributos.
	static map< string, string > ImNodeToSymbol(InnerModelNode* node);

public:

	//return the symbolID of the node that includes the innerModel name n.
	static int findSymbolIDWithInnerModelName(AGMModel::SPtr &m, QString n);

	//return only the symbolic part of the graph
	static AGMModel::SPtr extractSymbolicGraph( AGMModel::SPtr &worldModel);

	//this three functions work together.
	static InnerModel* extractInnerModel( AGMModel::SPtr &worldModel, QString imNodeName="world", bool ignoreMeshes=false);

	//udpate AGM with the info contains in *im
	static void updateAgmWithInnerModel( AGMModel::SPtr &worldModel, InnerModel* im);

	//Actualiza cada simbolo de agm que es de innermodel con la información de innermodel
	//todos los nodos de innermodel tiene que tene su correspondiente en AGM
	static void updateAgmWithInnerModelAndPublish( AGMModel::SPtr &worldModel, InnerModel* im, AGMExecutivePrx &agmexecitive_proxy);

	//update innermodel node from edges
	static void updateImNodeFromEdge( AGMModel::SPtr &worldModel, AGMModelEdge edge, InnerModel *innerModel);
	static void updateImNodeFromEdge( AGMModel::SPtr &worldModel, const RoboCompAGMWorldModel::Edge& edge, InnerModel *innerModel);

	//DEPRECATED
	//Insert innermodel in AGM graph matching nodes from innerModel to their correspondent symbols.
	//este método empareja diferentes nodos a diferentes símbolos
	//excesivo
	static  void include_im(AGMModel::SPtr &worldModel, QHash<QString, int32_t>  match, InnerModel *im);

	//incluye innermodel desde el símbolo symbolID
	//si el símbolo no tiene el atributo imType se crea. Con la unión del type + _ + symbolID
	static void includeInnerModel(AGMModel::SPtr &worldModel, int symbolID, InnerModel *im);

	//Dado un innerModel (impTmp) elimina de AGM todos los symbolos que lo forman.
	//Para borrar subgrafos, como la persona por ejemplo.
	static void  removeInnerModel(AGMModel::SPtr &worldModel, InnerModel*imTmp);

	//find a loop from a symbolID throught a fixed linkType
	static void checkLoop( AGMModel::SPtr &worldModel, int& symbolID, QList< int >& visited, string linkType, bool& loop);

	//Deprecated (no se cual fue su objetivo)
	//static AGMModel::SPtr remove_ImOriginal(string agmFilePath, string imFilePath);

	//list of id symbols from a symbolID throught a specific linkType
// 	QList< int > getLinkedID(int symbolID, string linkType);



};

#else

#endif

#endif // AGMINNER_H
