/*
 * 
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
 *  
 */
 

#include "agmInner.h"
#include <innermodel/innermodel.h>

AgmInner::AgmInner()
{
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel"; 
	innerModel = new InnerModel();
}

void AgmInner::setWorld(AGMModel::SPtr model)
{
	worldModel = model;
}
AGMModel::SPtr AgmInner::getWorld()
{
	return worldModel;
}


AgmInner::~AgmInner()
{

}


///functions
/**
 * @brief Search the name of the innermodel node,(the name is the unique key for innerModel ), inside de AGM Model. The innermodel id is stored in the attribute "name" of each symbol. 
 * It is found, return the id of the symbol, the unique key for AGMSymbols, otherwise returns -1.
 * 
 * @param n value of the attribute field name...
 * @return symbol ID, -1 if it is not found
 */
int AgmInner::findName(QString n)
{
	for (uint32_t i=0; i<worldModel->symbols.size(); ++i)
	{	
		if (worldModel->symbols[i]->attributes.find("name") != worldModel->symbols[i]->attributes.end() )
		{
			if (worldModel->symbols[i]->attributes["name"] == n.toStdString() )
			{
// 				qDebug()<<"findName: FOUND"<<n<<worldModel->symbols[i]->identifier;
				return worldModel->symbols[i]->identifier;
			}
		}
	}	
// 	qDebug()<<"findName: NO ENCONTRADO"<<n<<-1;
	return -1;
}

/**
 * @brief 1 Find transform node reading his attribute "name" to get his corresponding symbol ID
 * 2 Go through rt edge starting from the agm symbol found before. Format Link a-RT->b
 * 3 Update the new InnerModel with the information stored in the edge
 * 
 * @param imNodeName InnerModelNode name to start the path
 * @return InnerModel* tree from InnerModelNode name
 */
InnerModel* AgmInner::extractInnerModel(QString imNodeName)
{
	
	
	InnerModel *imNew = new InnerModel() ;

	int symbolID = findName(imNodeName);
// 	symbolID=5;
	if (symbolID > -1)
		recorrer(imNew, symbolID);
	
	return imNew;
}



/**
 * @brief Recorre el grafo por los enlaces RT desde el symbolID creando un innerModel equivalente
 * 
 * @param imNew Contiene el Innermodel equivalente generado
 * @param symbolID ID del symbolo punto de partida
 * @return void
 */
void AgmInner::recorrer(InnerModel* imNew, int& symbolID)
{
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(symbolID);
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==symbolID )
		{
			int second = (*edge_itr)->getSymbolPair().second;
			edgeToInnerModel((*edge_itr),imNew);
			recorrer(imNew,second);
		}
	}
}

void AgmInner::checkLoop(int& symbolID, QList<int> &visited, string linkType, bool &loop)
{
	if (visited.contains(symbolID) )
	{
		loop=true;
		visited.append(symbolID);
		return;
	}
	else
		visited.append(symbolID);
	
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(symbolID);
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
		if ((*edge_itr)->getLabel() == linkType && (*edge_itr)->getSymbolPair().first==symbolID )
		{
			int second = (*edge_itr)->getSymbolPair().second;
			qDebug()<<symbolID<<"--"<<QString::fromStdString(linkType)<<"-->"<<second;
			qDebug()<<"\tvisited"<<visited;									
			checkLoop(second,visited, linkType,loop);
		}
	}
	
}

/**
 * @brief ..transform the information contains in an AGM edge in two InnerModelNode, adding the information stored in the label RT 
 * to the father's transformation .
 * 
 * @param edge AGMModelEdge...
 * @param imNew ...
 * @return void
 */
void AgmInner::edgeToInnerModel(AGMModelEdge edge, InnerModel* imNew) 
{
	InnerModelNode* nodeA = NULL;
	InnerModelNode* nodeB = NULL;

	int first = edge->getSymbolPair().first;
	int second = edge->getSymbolPair().second;	

	const AGMModelSymbol::SPtr &symbolA = worldModel->getSymbol(first);
	const AGMModelSymbol::SPtr &symbolB = worldModel->getSymbol(second);

	QString nameA = QString::fromStdString(symbolA->attributes["name"]);
	QString nameB = QString::fromStdString(symbolB->attributes["name"]);
	
// 	qDebug()<<"insertar en new InnerModel "<<nameA<<"--"<< QString::fromStdString ( edge->getLabel() ) <<"-->"<<nameB;//<<tx<<ty<<tz<<rx<<ry<<rz;
// 	qDebug()<<"equivalente al enlace en AGM "<<QString::fromStdString (symbolA->toString())<<"--"<< QString::fromStdString ( edge->getLabel() ) <<"-->"<<QString::fromStdString (symbolB->toString());//<<tx<<ty<<tz<<rx<<ry<<rz;
	
	
	//node father
	nodeA=imNew->getNode(nameA);
	//entiendo que sino exite lo cuelgo del root, Estará vacio...
	if (nodeA==NULL)
	{
		nodeA = imNew->newTransform(nameA, "static",imNew->getRoot());
		imNew->getRoot()->addChild(nodeA);
// 		qDebug()<<"NODE A ERA NULL NO EXISTIA"<<nameA<<"node A->print(verbose):";
// 		nodeA->print(true);
// 		qDebug()<<"---------";
	}
	
	float tx,ty,tz,rx,ry,rz;
	tx=ty=tz=rx=ry=rz=0.;
	tx = str2float(edge->attributes["tx"]);
	ty = str2float(edge->attributes["ty"]);
	tz = str2float(edge->attributes["tz"]);

	rx = str2float(edge->attributes["rx"]);
	ry = str2float(edge->attributes["ry"]);
	rz = str2float(edge->attributes["rz"]);
	nodeB = imNew->newTransform (nameB, "static",nodeA,tx,ty,tz,rx,ry,rz);	
	if (nodeB==NULL)
		qFatal("MAAAAAL edgeToInnerModel() nodeB == null ");
	nodeA->addChild(nodeB);

}

QList<int> AgmInner::getLinkedID (int symbolID, string linkType)
{
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(symbolID);
	QList<int> l;
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";			
		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
		if ((*edge_itr)->getLabel() == linkType && (*edge_itr)->getSymbolPair().first==symbolID )
		{
			
			l.append((*edge_itr)->getSymbolPair().second);
		}
	}
	return l;
}

void AgmInner::updateAgmWithInnerModel(InnerModel* im)
{
	/// Vector of the edges that the model holds.
	std::vector<AGMModelEdge> edges;
	std::cout << "myvector contains:"<<worldModel->edges.size();

	///tal vez sería bueno recorrer primero innerModel con include_im y crear attributes name por cada symbolo, pq puede haberse insertado algun nodo nuevo.
	for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
	{
		std::cout << ' ' << (*it)->toString(worldModel);
		if ((*it)->getLabel()=="RT" )
		{
			string songName;
			
			//obtengo del symbol hijo el atribute name
			songName= (worldModel->getSymbol((*it)->getSymbolPair().second))->attributes["name"];
			std::cout <<"\t"<<songName<<"\n";
			try 
			{
				InnerModelTransform *node= im->getTransform (QString::fromStdString(songName));
				(*it)->setAttribute("tx",float2str( node->getTr().x() ));
				(*it)->setAttribute("ty",float2str( node->getTr().y() ));
				(*it)->setAttribute("tz",float2str( node->getTr().z() ));
				(*it)->setAttribute("rx",float2str( node->getRxValue()));
				(*it)->setAttribute("ry",float2str( node->getRyValue()));
				(*it)->setAttribute("rz",float2str( node->getRzValue()));
			}
			catch (QString error)
			{
				qDebug()<<"EXCEPTION"<<error;
			}
			
			
		}
		std::cout << '\n';
	}
// 	myvector contains:9 isKitchen object_3 roomSt_4 room object_3 roomSt_4 explored object_3 roomSt_4 free robot_1 robotSt_2 in robot_1 object_3 in world_20 robot_1 RT world_20 robot_1 RT world_20 transform_21 RT world_20 transform_22

}



