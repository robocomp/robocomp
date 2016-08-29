#ifndef WORLDMODELDRAWER_H
#define WORLDMODELDRAWER_H

#include <QObject>
#include <QTableWidget>
#include <QHash>
#include <stdlib.h>
#include <string>

#ifndef Q_MOC_RUN
#include <agm_modelPrinter.h>
#include <agm_misc_functions.h>
#include <agm_model.h>
#include <agm_modelEdge.h>
#include <agm_modelSymbols.h>
#endif

#include <rcdraw/rcdraw.h>




#define SPRING_LENGTH 17.
#define HOOKES_CONSTANT 1.5
#define FRICTION 0.85
#define FIELD_FORCE_MULTIPLIER 700000.


/*!
 * @brief AGMModel drawing class
 *
 * @ingroup CPPAPI
 * 
 * 
 */
class AGMModelDrawer : QObject
{
Q_OBJECT
public:
	/// Constructor. It is parametrized with a RCDraw object and an <strong>optional</strong> QTableWidget object (used to show nodes' attributes).
	AGMModelDrawer(RCDraw *drawer_, QTableWidget *tableWidget_=NULL)
	{
		drawer = drawer_;
		drawer->setZoomMultiplier(0.91);
		tableWidget = tableWidget_;
		modified = false;
		showInner = false;
		showRobot = false;
		showMesh = false;
		showPlane = false;
		visited.clear();
		meshList.clear();
		planeList.clear();
		connect(drawer, SIGNAL(newCoor(QPointF)), this, SLOT(clickedNode(QPointF)));
	}
	void setInterest(std::string item)
	{
		interest = item;
		drawTable();
	}

	/// This method updates the widget with the current model ('w' vairable).
	void update(const AGMModel::SPtr &w)
	{
		
		drawer->autoResize(true);
		
		mutex.lock();
		model = AGMModel::SPtr(new AGMModel(w));
		updateStructure();
// 		std::cout<<"rerecalculatePositions()\n";
		recalculatePositions();
// 		std::cout<<"draw\n";
		draw();
// 		std::cout<<"draw->updater\n";
		drawer->update();
// 		std::cout<<"en update\n";
		mutex.unlock();
	}
	void setShowMesh(bool s=false)
	{
		showMesh=s;
		meshList.clear();
		for (uint32_t i=0; i<model->symbols.size(); ++i)
		{
			QString type =QString::fromStdString( model->symbols[i]->symbolType );
			if (type  == "mesh")
			{
				meshList.append(model->symbols[i]->identifier);
			}
		}
	}
	void setShowPlane (bool s=false)
	{
		showPlane=s;
		planeList.clear();
		for (uint32_t i=0; i<model->symbols.size(); ++i)
		{
			QString type =QString::fromStdString( model->symbols[i]->symbolType );
			if (type  == "plane")
			{
				planeList.append(model->symbols[i]->identifier);
			}
		}
	}
	void setShowInnerModel(bool s=false)
	{
		showInner = s;
	}
	
	void setShowRobot( bool s=false)
	{
		if (s==true)
		{
			showRobot = s;
			return;
		}
		
		visited.clear();
		bool loop=false;
		int symbolID=1;
		try
		{
			model->getSymbolByIdentifier(symbolID);
		}
		catch(AGMModelException e)
		{
			std::cout<<"robot, symbol id 1 not found "<<e.what()<<"\n";
		}
		listOfSymbolThroughLinkType (symbolID,visited,"RT",loop);
		if (loop)
		{
			std::cout<<"mission setShowRobot loop starting from robot symbol 1\n";		   
			exit(-1);
		}
		showRobot=s;
		
// 		std::cout<<"visited.size() "<<visited.size()<<"\n";
// 		for (int i=0; i<visited.size();i++)
// 		{
// 			std::cout<<"id "<<visited.at(i)<<" symbol "<<model->getSymbolByIdentifier(visited.at(i))->toString();
// 		}
// 		
// 		
// 		std::cout<<"\n setShowRobot "<<showRobot<<"\n\n";
// 		std::cout<<" fin setShowRobot \n\n";
		
		
		
	}

	void listOfSymbolThroughLinkType( int& symbolID, QList<int> &visited, std::string linkType, bool &loop)
	{
		if (visited.contains(symbolID) )
		{
			loop=true;
			visited.append(symbolID);
			return;
		}
		else
			visited.append(symbolID);
		try
		{
			const AGMModelSymbol::SPtr &symbol = model->getSymbol(symbolID);
			for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(model); edge_itr!=symbol->edgesEnd(model); edge_itr++)
			{
				//std::cout<<(*edge_itr).toString(model)<<"\n";
				//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
				if ((*edge_itr)->getLabel() == linkType && (*edge_itr)->getSymbolPair().first==symbolID )
				{
					int second = (*edge_itr)->getSymbolPair().second;
					//std::cout<<symbolID<<" -- "<<linkType<<" --> "<<second;				
					listOfSymbolThroughLinkType(second,visited, linkType,loop);
				}
			}
		}
		catch(...){}
	}
		// Draws attribute table, nodes and edge.
	void drawTable()
	{
		int32_t index2=-1;
		std::map<std::string,std::string> attributes;
		for (uint32_t i=0; i<model->edges.size(); ++i)
		{
// 			std::cout<<model->edges[i].toString(model)<<"\n";
			if (model->edges[i].toString(model)==interest)
			{
				index2 = i;
				attributes= model->edges[index2]->attributes;
				break;
			}
		}
		
		
		if (index2==-1)
		{
			for (uint32_t i=0; i<model->symbols.size(); ++i)
			{
				if (model->symbols[i]->toString() == interest)
				{
					index2 = i;
					attributes= model->symbols[index2]->attributes;
					break;
				}
			}
		}
		if (index2 != -1 )
		{
			 
			tableWidget->setColumnCount(2);
			tableWidget->setRowCount(attributes.size()+1);
			QTableWidgetItem *ti;

			ti = tableWidget->item(0, 0);
			if (ti == NULL) { ti = new QTableWidgetItem(); tableWidget->setItem(0, 0, ti);}
			ti->setText(QString("ID"));

			ti = tableWidget->item(0, 1);
			if (ti == NULL) { ti = new QTableWidgetItem(); tableWidget->setItem(0, 1, ti);}
// 			ti->setText(QString::fromStdString(model->symbols[index2]->toString()));
			ti->setText(QString::fromStdString(interest));

			int row = 1;
			for (std::map<std::string,std::string>::iterator iter = attributes.begin(); iter != attributes.end(); iter++, row++)
			{
				ti = tableWidget->item(row, 0);
				if (ti == NULL) { ti = new QTableWidgetItem(); tableWidget->setItem(row, 0, ti);}
				ti->setText(QString::fromStdString( iter->first ));

				ti = tableWidget->item(row, 1);
				if (ti == NULL) { ti = new QTableWidgetItem(); tableWidget->setItem(row, 1, ti);}
				ti->setText(QString::fromStdString( iter->second ));
			}
		}
				
		else
		{
			if (tableWidget != NULL) tableWidget->clear();
		}
		
	}

	/// Move the nodes according to a edges-attraction / nodes-repulsion simulation.
	void recalculatePositions()
	{
		static QTime timer = QTime::currentTime();
		float time = double(timer.elapsed())/1000.;
		timer = QTime::currentTime();
		if (time > 500) time = 500;

		// Compute forces and integrate velocities, storing updated velocities in nodes[n].vel[0-1]
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			if (nodes[n].show==false)
				continue;
			int32_t i[2];
			double forceX=0., forceY=0.;
			for (uint32_t n2=0; n2<nodes.size(); n2++)
			{
				if (n == n2 or nodes[n2].show==false)
				{
					continue;
				}
				for (int d=0; d<2; d++)
					i[d] = nodes[n].pos[d] - nodes[n2].pos[d];
				if (i[0] == 0 and i[1] == 0)
				{
					nodes[n2].pos[0]++;
					nodes[n2].pos[1]++;
					for (int d=0; d<2; d++)
						i[d] = nodes[n].pos[d] - nodes[n2].pos[d];
				}
				float angle = atan2(i[1], i[0]);
				float dist1 = pow((abs((i[1]*i[1]) + (i[0]*i[0]))), 0.5);
				if (dist1 < SPRING_LENGTH)
					dist1 = SPRING_LENGTH;
				float dist2 = pow(dist1, 2.);
				float force = FIELD_FORCE_MULTIPLIER / dist2;
				forceX += force * cos(angle);
				forceY += force * sin(angle);
			}
			for (uint32_t n2=0; n2<nodes.size(); n2++)
			{
				if(std::find(nodes[n].edges.begin(), nodes[n].edges.end(), n2) != nodes[n].edges.end())
				{
					for (int d=0; d<2; d++)
						i[d] = nodes[n].pos[d] - nodes[n2].pos[d];
					float angle = atan2(i[1], i[0]);
					float force = sqrt(abs((i[1]*i[1]) + (i[0]*i[0])));
					if (force <= SPRING_LENGTH) continue;
					force -= SPRING_LENGTH;
					force = force * HOOKES_CONSTANT;
					forceX -= force * cos(angle);
					forceY -= force * sin(angle);
				}
			}

			nodes[n].vel[0] = (nodes[n].vel[0] + (forceX*time))*FRICTION;
			nodes[n].vel[1] = (nodes[n].vel[1] + (forceY*time))*FRICTION;
			float v = sqrt((nodes[n].vel[0]*nodes[n].vel[0]) + (nodes[n].vel[1]*nodes[n].vel[1]));
			float MAX = 50;
			if (v > MAX)
			{
				nodes[n].vel[0] = nodes[n].vel[0] / v * MAX;
				nodes[n].vel[1] = nodes[n].vel[1] / v * MAX;
			}
		}
		// Integrate velocities, storing the result in nodes[n].pos
		// Also, implement friction by multipling velocities by FRICTION
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			if (nodes[n].show==false)
				continue;
			for (int d=0; d<2; d++)
			{
				nodes[n].pos[d] += nodes[n].vel[d];
			}
		}

		if (nodes.size() > 0)
		{
			double totalX = 0;
			double totalY = 0;
			for (uint32_t n=0; n<nodes.size(); n++)
			{
				if (nodes[n].show==false)
					continue;
				totalX += nodes[n].pos[0];
				totalY += nodes[n].pos[1];
			}
			
			double nodesSizeShow=0;
			for (uint32_t n=0; n<nodes.size(); n++)
			{
				if (nodes[n].show==false)
					nodesSizeShow++;
			}
			
			totalX /= nodes.size()-nodesSizeShow;
			totalY /= nodes.size()-nodesSizeShow;
			for (uint32_t n=0; n<nodes.size(); n++)
			{
				if (nodes[n].show==false)
					continue;
				nodes[n].pos[0] -= totalX;
				nodes[n].pos[1] -= totalY;
			}
		}
	}


private:
	/// Inspects the current model to detect significant changes.
	void updateStructure()
	{
		// Push back new nodes
		
		for (uint32_t e1=0; e1<model->symbols.size(); e1++)
		{	
			
				
			bool found = false;
			for (uint32_t e2=0; e2<nodes.size(); e2++)
			{
				if (nodes[e2].name == model->symbols[e1]->toString())
				{
					found = true;
					nodes[e2].show=true;
					break;
				}
			}
			if (not found)
			{
				GraphicalNodeInfo node;
				node.name = model->symbols[e1]->toString();
				node.type = model->symbols[e1]->symbolType;
				node.identifier = model->symbols[e1]->identifier;				
				node.show=true;
				for (int d=0; d<2; d++)
				{
					node.pos[d] = (100.*rand())/RAND_MAX - 50.;
					node.vel[d] = 0;
				}
				node.labelsPositions.clear();
				nodes.push_back(node);
			}
			
			
				
		}
		// Remove deleted nodes
		for (uint32_t e1=0; e1<nodes.size();)
		{
			bool found = false;
			for (uint32_t e2=0; e2<model->symbols.size(); e2++)
			{
				if (nodes[e1].name == model->symbols[e2]->toString())
				{
					found = true;
					break;
				}
			}
			if (not found)
				nodes.erase(nodes.begin() + e1);
			else
				e1++;
		}
		// Clear edges
		for (uint32_t e=0; e<nodes.size();e++)
		{
			nodes[e].edges.clear();
			nodes[e].edgesOriented.clear();
			nodes[e].edgesNames.clear();
			nodes[e].labelsPositions.clear();
		}
		// Push back edges again
		for (uint32_t e=0; e<model->edges.size(); e++)
		{	
			if (model->edges[e].linking=="RT" && showInner==false)
				continue;
			if (model->edges[e].linking=="RT" && showRobot==false && showInner==true)
			{
				int first =model->edges[e].symbolPair.first;
				int second =model->edges[e].symbolPair.second;
				if ( visited.contains(first) and visited.contains(second) )
					continue;
			}
			if (model->edges[e].linking=="RT" &&  showInner==true && showMesh ==false)
			{
// 				int first =model->edges[e].symbolPair.first;
				int second =model->edges[e].symbolPair.second;
				if ( meshList.contains(second) )
					continue;
			}
			if (model->edges[e].linking=="RT"  && showInner==true && showPlane ==false)
			{
// 				int first =model->edges[e].symbolPair.first;
				int second =model->edges[e].symbolPair.second;
				if ( planeList.contains(second) )
					continue;
			}
			

			int f = model->getIndexByIdentifier(model->edges[e].symbolPair.first);
			int s = model->getIndexByIdentifier(model->edges[e].symbolPair.second);
			if (s<0 or f<0)
			{
// 				printf("%d --> %d\n", model->getIndexByIdentifier(model->edges[e].symbolPair.first), model->getIndexByIdentifier(model->edges[e].symbolPair.second));
				continue;	
			}
			std::string first  = model->symbols[f]->toString();
			std::string second = model->symbols[s]->toString();
			int32_t idx1=-1;
			int32_t idx2=-1;
			for (uint32_t n=0; n<nodes.size(); n++)
			{
				if (nodes[n].name == first)
					idx1 = n;
				if (nodes[n].name == second)
					idx2 = n;
			}
			if (idx1 > -1 and idx2 > -1)
			{
				nodes[idx1].edges.push_back(idx2);
				nodes[idx2].edges.push_back(idx1);

				nodes[idx1].edgesOriented.push_back(idx2);
				nodes[idx1].edgesNames.push_back(model->edges[e].linking);
			}
			else
			{
				printf("We had a link whose nodes where not found?!? (%s --> %s)\n", first.c_str(), second.c_str());
				exit(-1);
			}
		}
		
		// Remove nodes with only a nodes
		//borra los nodos que solo tienen enlaces RT (deben estar solos)
		if (showInner==false)
		{
			//std::cout << "\n\n ******************************** \n";			
			for (uint32_t e1=0; e1<nodes.size(); e1++)
			{
				//nodo solo
				if (nodes.at(e1).edges.empty())
				{
// 					std::cout << "\t SHOWWINNNER must be erase, e1: "<< e1<<" "<< nodes[e1].name<<" labels: "<<nodes[e1].edgesNames.size()<<" \n";
					nodes[e1].show=false;
				}
			}
		}
		//se muestrra innermodel y no se quiere mostrar el robot
		//borro los nodos del robot
		else if (showRobot==false)
		{
// 			std::cout << "\n\n *************  statr bucle showRobot "<<nodes.size()<<" *******************  \n";			
			for (uint32_t e1=0; e1<nodes.size();e1++)
			{
				
					//nodo solo del robot
					if (nodes.at(e1).edges.empty() and visited.contains(nodes[e1].identifier))
					{					
// 						std::cout << "\t must be erase, e1: "<< e1<<" "<< nodes[e1].name<<" labels: "<<nodes[e1].edgesNames.size()<<" \n";
						nodes[e1].show=false;
					}
			}
// 			std::cout << "\n\n ************* FIN bucle showRobot "<<nodes.size()<<" ******************* \n";			
			
		}
		if (showInner and showMesh == false)
		{
			for (uint32_t e1=0; e1<nodes.size();e1++)
			{
				if ( meshList.contains(nodes[e1].identifier))
				{	
					nodes[e1].show=false;
				}
			}
		}
		if (showInner and showPlane == false)
		{
			for (uint32_t e1=0; e1<nodes.size();e1++)
			{
				if ( planeList.contains(nodes[e1].identifier))
				{	
					nodes[e1].show=false;
				}
			}
		}
		modified = true;
	}

	// Actually draw the model in the widget.
	void draw()
	{
		const float radius = 25.;

		QPointF c = drawer->getWindow().center();
		int wW2 = c.x();
		int wH2 = c.y();
// 		qDebug()<<"\t\tINIT DRAW";
		// Draw links
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			
			if (nodes[n].show==false)
				continue;
			
			if (modified)
			{
				/*printf("IDENTIFIER %s  index:%d (LINKS:%d,%d)\n", nodes[n].name.c_str(), n, (int)nodes[n].edgesOriented.size(), (int)nodes[n].edgesNames.size());*/
			}
			for (uint32_t e=0; e<nodes[n].edgesOriented.size(); e++)
			{
				int32_t o1 = n;
				int32_t d1 = nodes[n].edgesOriented[e];
				int pos = 0;
				int linkGroupCount = 0;
				for (uint32_t n2=0; n2<nodes.size(); n2++)
				{
					for (uint32_t e2=0; e2<nodes[n2].edgesOriented.size(); e2++)
					{
						if (n==n2 and e==e2)
						{
							pos = linkGroupCount;
						}
						int32_t o2 = n2;
						int32_t d2 = nodes[n2].edgesOriented[e2];
						if ((o1==o2 and d1==d2) or (o1==d2 and d1==o2))
						{
							linkGroupCount += 1;
						}
					}
				}
				
				QPointF p1 = QPointF(nodes[            n            ].pos[0]+wW2, wH2-nodes[            n            ].pos[1]);
				QPointF p2 = QPointF(nodes[nodes[n].edgesOriented[e]].pos[0]+wW2, wH2-nodes[nodes[n].edgesOriented[e]].pos[1]);
				QPointF inc = (p2 - p1);
				inc /= sqrt(inc.x()*inc.x() + inc.y()*inc.y());
				p1 = p1 + radius*inc;
				p2 = p2 - radius*inc;
				
				// Link itself
				drawer->drawLine(QLineF(p1, p2), QColor(0, 0, 0));
				// Arrow
				float angle = -atan2(inc.y(), inc.x());
				for (float a = -0.4; a<=0.4; a+=0.05)
				{
					QPointF pr = QPointF(p2.x()-(radius*0.5)*cos(angle+a), p2.y()+(radius*0.5)*sin(angle+a));
					drawer->drawLine(QLineF(p2,  pr), QColor(0, 0, 0));
				}
				
				// Text
				int32_t linkHeight = 16;
				int32_t linkGroupBase = (-linkGroupCount+1)*linkHeight/2;
				QPointF reLl(0, linkHeight*pos + linkGroupBase);
				QPointF labelTextPosition=p1*0.6+p2*0.4-reLl;
				
				//Label
				//destiny e in edgesOriented
// 				std::cout<<"nodes[n].name "<<nodes[n].name<<"\n";
// 				qDebug()<<"nodes[n].edgesOriented.size()"<<nodes[n].edgesOriented.size();
// 				qDebug()<<"destiny e in edgesOriented "<<e;				
				int32_t dst=nodes[n].edgesOriented[e];
// 				qDebug()<<"dst"<<dst;
				std::string nameSymbolDst=nodes[dst].name;				
// 				std::cout<<"nameSymbolDst "<<nameSymbolDst<<"\n";
				std::string stringStream;
				stringStream =nodes[n].edgesNames[e] +" "+nodes[n].name+" "+nameSymbolDst;
// 				std::cout<<"stringStream "<<stringStream<<"\n";
				QString labelKey = QString::fromStdString( stringStream );
				if (nodes[n].labelsPositions.contains(labelKey) == true )
				{   
					qDebug()<<"edge repeated"<<labelKey;
					qDebug()<<"nodes["<<n<<"].labelsPositions"<<nodes[n].labelsPositions;
					qFatal("Error:: Two equal edges!!!");
				}
				else 
				{
					nodes[n].labelsPositions[labelKey]=labelTextPosition;
					drawer->drawText(labelTextPosition, QString::fromStdString(nodes[n].edgesNames[e]), 10, QColor(255), true, QColor(127,127,127,127));
				}
// 				qDebug()<<"\t----";
// 				if (modified) printf("  link[%s] id(%s-->%s) idx(%d-->%d)\n", nodes[n].edgesNames[e].c_str(), nodes[n].name.c_str() , nodes[nodes[n].edgesOriented[e]].name.c_str(), n, nodes[n].edgesOriented[e]);
			}
		}
		// Draw nodes
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			if (nodes[n].show==false)
				continue;
			const QPointF p = QPointF(nodes[n].pos[0]+wW2, wH2-nodes[n].pos[1]);
			drawer->drawEllipse(p, radius, radius, QColor(255, 0, 0), true);
			drawer->drawText(p+QPointF(0,+10), QString::fromStdString(nodes[n].type), 10, QColor(255), true);
			drawer->drawText(p+QPointF(0,-7), QString::number(nodes[n].identifier), 10, QColor(255), true);
		}
		modified = false;
		
		if (tableWidget != NULL) drawTable();
// 		qDebug()<<"\t\tEND DRAW";
	}

private:
	// Internal representation for nodes.
	struct GraphicalNodeInfo
	{
		std::string name;
		std::string type;
		int32_t identifier;
		bool show;
		int32_t pos[2];
		float vel[2];
		std::vector<uint32_t> edges;
		std::vector<uint32_t> edgesOriented;
		std::vector<std::string> edgesNames;
		QHash<QString, QPointF> labelsPositions;
		
	};

private:
	bool modified;
	RCDraw *drawer;
	QMutex mutex;
	std::vector<GraphicalNodeInfo> nodes;
	AGMModel::SPtr model;
	QTableWidget *tableWidget;
	std::string interest;
	bool showInner;
	bool showRobot;
	bool showMesh;
	bool showPlane;
	QList<int> visited;
	QList<int> meshList;
	QList<int> planeList;
	
	


	void print ()
	{
		qDebug()<<"\nAGMModelPrinter::printWorld(model)\n";
		AGMModelPrinter::printWorld(model);
// 		
		qDebug()<<"\nAGMModelDrawer::Print Graphical Structure\n";
		qDebug()<<"nodes.size()"<<nodes.size();		
		
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			
			qDebug()<<"\nNode"<<n;
			std::cout<<"nodes["<<n<<"].name "<<nodes[n].name<<std::endl;
			std::cout<<"nodes["<<n<<"].type "<<nodes[n].type<<std::endl;
			std::cout<<"nodes["<<n<<"].type "<<nodes[n].identifier<<std::endl;
			qDebug()<<"nodes["<<n<<"].pos"<<nodes[n].pos[0]<<nodes[n].pos[1];
			qDebug()<<"nodes["<<n<<"].edgesNames.size()"<<nodes[n].edgesNames.size();
			for (uint32_t e=0; e<nodes[n].edgesNames.size(); e++)
			{
				std::cout<<"\tnodes["<<n<<"].edgesNames ["<<e<<"]"<<nodes[n].edgesNames[e]<<"\n";
			}
			///edges
			qDebug()<<"nodes["<<n<<"].edges.size()"<<nodes[n].edges.size();
			for (uint32_t e=0; e<nodes[n].edges.size(); e++)
			{
				std::cout<<"\tnodes["<< n <<"].edges ["<<e<<"]"<<nodes[n].edges[e]<<"\n";
			}
			///edgesOriented
			qDebug()<<"nodes["<<n<<"].edgesOriented.size()"<<nodes[n].edgesOriented.size();
			for (uint32_t e=0; e<nodes[n].edgesOriented.size(); e++)
			{
				std::cout<<"\tnodes["<< n <<"].edgesOriented ["<<e<<"]"<<nodes[n].edgesOriented[e]<<"\n";
			}
			///labelsPositions
			qDebug()<<"nodes[n].labelsPositions"<<nodes[n].labelsPositions.size();
			foreach (const QString &name, nodes[n].labelsPositions.keys())
			{
				qDebug() <<"\t"<< name << ":" << nodes[n].labelsPositions.value(name);
			}
			
		}
	}

public slots:
	// Updates selected node in the attribute table according to the provided coordinates.
	void clickedNode(QPointF pC)
	{
		if (tableWidget == NULL) return;
		QPointF c = drawer->getWindow().center();
		
		int wW2 = c.x();
		int wH2 = c.y();
		float x = pC.x();
		float y = pC.y();
		int32_t index=-1;
		int32_t indexLink = -1;
		float minDist=-1;
		float minDistLink = -1;
		QString clickedLabel;
		for (uint32_t n=0; n<nodes.size(); n++)
		{
			
			const QPointF p = QPointF(nodes[n].pos[0]+wW2, wH2+nodes[n].pos[1]);
			const float dist = sqrt( pow(p.x()-x, 2)+pow(p.y()-y, 2));
// 			qDebug()<<QString::fromStdString(nodes[n].name)<<"node pos ("<<nodes[n].pos[0]<<","<<nodes[n].pos[1]<<")"<<"p"<<p<<"dist"<<dist;
			//distance to node
			if ( dist < 20.)
			{
				if (dist < minDist or minDist < 0)
				{
					index = n;
					minDist = dist;
				}
			}
			//distance to label
			foreach (const QString &str, nodes[n].labelsPositions.keys())
 			{
				QPointF labelTextPosition = nodes[n].labelsPositions.value(str);
				const QPointF pLabel = QPointF(labelTextPosition.x(), -1*(labelTextPosition.y()-drawer->getWindow().height()));
				const float distLabel = sqrt( pow(pLabel.x()-x, 2)+pow(pLabel.y()-y, 2));
// 				qDebug()<<"\tlabel"<<str<<":"<<nodes[n].labelsPositions.value(str)<<"pLabel"<<pLabel<<"distLabel"<<distLabel;
				
				if ( distLabel < 20.)
				{
					if (distLabel < minDistLink or minDistLink < 0)
					{
						indexLink = n;
						minDistLink = distLabel;
						clickedLabel =str;
					}
				}
			}

		}
		if (index > -1)
		{
			interest = nodes[index].name;
		}
		else if (indexLink > -1)
		{
			
			interest = clickedLabel.toStdString();
		}
		
	}
};


#endif

