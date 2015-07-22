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
	//idea...
	///innerModel= extractInnerModel();
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
 * @brief Search the name of the innermodel node,(the name is the unique key for innerModel ), in the AGM Model parameter. The innermodel id is stored in the attribute "name" of each symbol. 
 * It is found, return the id of the symbol, the unique key for AGMSymbols, otherwise returns -1.
 * 
 * @param n value of the attribute field name...
 * @return symbol ID, -1 if it is not found
 */
int AgmInner::findName(const AGMModel::SPtr &m, QString n)
{
	for (uint32_t i=0; i<m->symbols.size(); ++i)
	{	
		if (m->symbols[i]->attributes.find("name") != m->symbols[i]->attributes.end() )
		{
			if (m->symbols[i]->attributes["name"] == n.toStdString() )
			{
// 				qDebug()<<"findName: FOUND"<<n<<m->symbols[i]->identifier;
				return m->symbols[i]->identifier;
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
	//InnerModelNode* nodeB = NULL;

	int first = edge->getSymbolPair().first;
	int second = edge->getSymbolPair().second;	

	const AGMModelSymbol::SPtr &symbolA = worldModel->getSymbol(first);
	const AGMModelSymbol::SPtr &symbolB = worldModel->getSymbol(second);

	QString nameA = QString::fromStdString(symbolA->attributes["name"]);
	//QString nameB = QString::fromStdString(symbolB->attributes["name"]);
	
// 	qDebug()<<"insertar en new InnerModel "<<nameA<<"--"<< QString::fromStdString ( edge->getLabel() ) <<"-->"<<nameB;//<<tx<<ty<<tz<<rx<<ry<<rz;
// 	qDebug()<<"equivalente al enlace en AGM "<<QString::fromStdString (symbolA->toString())<<"--"<< QString::fromStdString ( edge->getLabel() ) <<"-->"<<QString::fromStdString (symbolB->toString());//<<tx<<ty<<tz<<rx<<ry<<rz;
	
	
	//node father
	nodeA=imNew->getNode(nameA);
	//entiendo que sino exite lo cuelgo del root, Estará vacio...
	if (nodeA==NULL)
	{
		//original
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
	
	//original
	//nodeB = imNew->newTransform (nameB, "static",nodeA,tx,ty,tz,rx,ry,rz);	
// 	if (nodeB==NULL)
// 		qFatal("MAAAAAL edgeToInnerModel() nodeB == null ");	
	//original
	//nodeA->addChild(nodeB);	
	
	try
	{
		insertSymbolToInnerModelNode(imNew,nodeA, symbolB,tx,ty,tz,rx,ry,rz);		
	}
	catch(string e)
	{
		std::cout<<e<<"\n";
		qFatal("insertSymbolToInnerModelNode");
	}
	
	

}
void AgmInner::insertSymbolToInnerModelNode(InnerModel* imNew,InnerModelNode *parentNode, AGMModelSymbol::SPtr s, float tx, float ty, float tz, float rx, float ry, float rz)
{
	
	QString nodeName = QString::fromStdString(s->attributes["name"]);
	std::cout<<"addding "<<s->attributes["name"];
	if (s->symbolType=="transform")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		QString engine =QString::fromStdString(s->attributes["engine"] );
		float mass =str2float( s->attributes["mass"] );
		
		try
		{
			InnerModelTransform* tf=imNew->newTransform (nodeName, engine,parentNode,tx,ty,tz,rx,ry,rz, mass);
			parentNode->addChild(tf);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}
	}
	else if (s->symbolType=="plane")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		float width, height, depth;		
		width= str2float(s->attributes["width"]); height= str2float(s->attributes["height"]); depth=str2float(s->attributes["depth"]);
		
		float nx,ny,nz,px,py,pz;
		nx =str2float( s->attributes["nx"] );ny =str2float( s->attributes["ny"] ); nz =str2float( s->attributes["nz"] );
		px =str2float( s->attributes["px"] );py =str2float( s->attributes["py"] ); pz =str2float( s->attributes["pz"] );
				
		bool collidable = false;
		if (s->attributes["collidable"]=="true")
			collidable = true;
		int repeat;
		repeat = str2int(s->attributes["repeat"]);
		QString texture;
		texture =QString::fromStdString(s->attributes["texture"]);				
		
		try
		{
			InnerModelPlane *plane=imNew->newPlane (nodeName,parentNode, texture,width,height,depth, repeat,nx,ny,nz,px,py,pz,collidable);
			parentNode->addChild(plane);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}

	}
	else if (s->symbolType=="mesh")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		
		QString meshPath;
		
		meshPath=QString::fromStdString(s->attributes["meshPath"]);
		float scalex,scaley,scalez;
		scalex=str2float(s->attributes["scalex"]);scaley=str2float(s->attributes["scaley"]);scalez=str2float(s->attributes["scalez"]);
		
		bool collidable = false;
		if (s->attributes["collidable"]=="true")
			collidable = true;
		
		//enum RenderingModes { NormalRendering=0, WireframeRendering=1};
		int render;
		render = InnerModelMesh::NormalRendering;
		if (s->attributes["render"] == "WireframeRendering")
			render = InnerModelMesh::WireframeRendering;
		try
		{
			InnerModelMesh *mesh=imNew->newMesh(nodeName,parentNode,meshPath,scalex,scaley,scalez,render,tx,ty,tz,rx,ry,rz, collidable);
			parentNode->addChild(mesh);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}
	}
	else if (s->symbolType=="rgbd")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		
		int port;		
		port =str2int(s->attributes["port"]);
		float noise, focal, height, width;
		noise = str2float(s->attributes["noise"]);
		focal = str2float(s->attributes["focal"]);
		height = str2float(s->attributes["height"]);
		width = str2float(s->attributes["width"]);				
		
		QString ifconfig;
		ifconfig=QString::fromStdString(s->attributes["ifconfig"]);
		
		try
		{
			InnerModelRGBD *rgbd=imNew->newRGBD(nodeName,parentNode,width,height,focal,noise,port,ifconfig);
			parentNode->addChild(rgbd);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}
	}
	else if (s->symbolType=="camera")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
				
		float focal, height, width;		
		focal = str2float(s->attributes["focal"]);
		height = str2float(s->attributes["height"]);
		width = str2float(s->attributes["width"]);				
		
		try
		{
			InnerModelCamera *c =imNew->newCamera(nodeName,parentNode,width,height,focal);
			parentNode->addChild(c);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}
	}
	else if (s->symbolType=="omniRobot")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		int port =str2int(s->attributes["port"]);
		float noise = str2float(s->attributes["noise"]);
		
		bool collide = false;
		if (s->attributes["collide"]=="true")
			collide = true;
		
		try
		{
			InnerModelOmniRobot *omni = imNew->newOmniRobot(nodeName,dynamic_cast<InnerModelTransform*>(parentNode),tx,ty,tz,rx,ry,rz,port,noise,collide);		
			parentNode->addChild(omni);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}
	}
	else if (s->symbolType=="joint")
	{
		std::cout<<"\t type: "<<s->symbolType <<"\n";
		//QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, ;
		//float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, 
		
		///ESTO NO SE USA EN NINGUN InnerModel.XML
		float lx =0.;// str2float(s->attributes["lx"] );
		float ly =0.;// str2float(s->attributes["ly"] );
		float lz =0.;// str2float(s->attributes["lz"] );
		float hx =0.;// str2float(s->attributes["hx"] );
		float hy =0.;// str2float(s->attributes["hy"] );
		float hz =0.;// str2float(s->attributes["hz"] );
		
		int port =str2int(s->attributes["port"]);		
		float min = str2float(s->attributes["min"]);
		float max = str2float(s->attributes["max"]);
		float home = str2float(s->attributes["home"]);
		string axis=s->attributes["axis"];
		
		
		try
		{
			InnerModelJoint *joint = innerModel->newJoint(nodeName,dynamic_cast<InnerModelTransform*>(parentNode),lx,ly,lz,hx,hy,hz,tx,ty,tz,rx,ry,rz,min,max,port,axis,home);		
			parentNode->addChild(joint);
		}
		catch (...)
		{
			qDebug()<<"Existe";
		}

	}
	else
	{
		QStringList l;
		l<<"robot"<<"object"<<"roomSt"<<"robotSt"<<"world"<<"objectSt";
	
		if ( l.contains(QString::fromStdString(s->symbolType)) )
		{
			std::cout<<"\t AGM SYMBOL, id imNode "<<s->toString()<<" type transform " <<"\n";
			InnerModelTransform* tf = imNew->newTransform (QString::fromStdString(s->toString()), "static",parentNode);	
			parentNode->addChild(tf);
		}
		
		else 
		{
			string err;			
			err = "\nsymbol "+ s->toString() + " the type is unknown or not implemented yet\n";
			std::cout<<err;
			throw err;
		}
		
	}
	
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

AGMModel::SPtr AgmInner::extractAGM()
{
	AGMModel::SPtr  agmModel = AGMModel::SPtr(new AGMModel(worldModel));
	QList<int>l;
	
	for (AGMModel::iterator symbol_itr=agmModel->begin(); symbol_itr!=agmModel->end(); symbol_itr++)
	{
		std::cout<<" delete? "<<(*symbol_itr)->toString()<<"\n";
		bool removeSymbol = true;
		for (AGMModelSymbol::iterator  edge_itr=symbol_itr->edgesBegin(agmModel); edge_itr!=symbol_itr->edgesEnd(agmModel); edge_itr++)
		{
			std::cout<<"\t"<<(*edge_itr).toString(agmModel);
			//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
			if ((*edge_itr)->getLabel() != "RT" )//&& ( (*edge_itr)->symbolPair.first==(*symbol_itr)->identifier) )
			{				
				removeSymbol=false;				
				break;
			}
		}
		if (removeSymbol)
		{	
			std::cout<<"\t yes delete "<<(*symbol_itr)->toString()<<" ";
			//std::cout<<agmModel->removeSymbol((*symbol_itr))<<"\n";
			l.append((*symbol_itr)->identifier);
		}
		std::cout<<"\n";
	}
	for (int i = 0; i < l.size(); ++i) 
	{
		agmModel->removeSymbol(l.at(i));
	}
	std::cout<<"agmModel->numberOfSymbols() "<<agmModel->numberOfSymbols()<<"\n";
	return agmModel;
}


/**
 * @brief Elimina del AGM en caliente, los nodos del innerModel original que no esten en el agm original. 
 * 
 * @param agmFilePath ...
 * @param imFilePath ...
 * @return AGMModel::SPtr limpio de innerModel (normalmente el propio del robot) pero con la info geometrica (RT) actualizadas para los symbolos del mundo "exterior" 
 */
AGMModel::SPtr  AgmInner::remove_ImOriginal(string agmFilePath, string imFilePath)
{
	AGMModel::SPtr  agmTmp = AGMModel::SPtr(new AGMModel());
	cout<<"agmFilePath: " <<agmFilePath<<"\n";
	
	
	AGMModelConverter::fromXMLToInternal(agmFilePath, agmTmp);	
	InnerModel *imTmp= new InnerModel (imFilePath); 
	
	std::cout<<agmTmp->numberOfSymbols()<<" "<<agmTmp->numberOfEdges()<<"\n";
	AGMModelPrinter::printWorld(agmTmp);
	imTmp->treePrint();

	//if imNode not in agmOriginal, remove the symbol associated to the node in the agmCaliente if exist
	foreach (QString n, imTmp->getIDKeys() )
	{
		qDebug()<<n;
		//findName in the original
		if (findName(agmTmp,n)==-1 )
		{
			int symbolID=findName(n);
			if (symbolID!=-1)
			{
				qDebug()<<"remove en el agmCaliente :"<<symbolID<<QString::fromStdString( worldModel->getSymbol(symbolID)->toString());
			}
		}
	}
	
	return agmTmp;
}



