/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	baseDatos = QSqlDatabase::addDatabase("QSQLITE");
	baseDatos.setDatabaseName("bd.db");
	if (!baseDatos.open())
            qDebug()<<"Base de Datos no pudo ser abierta";

	
	QStringList ltables=baseDatos.tables();
	if(!ltables.contains("logger"))
            qDebug()<<"creacion tabla"<< createTable();

	
	//preparacion de las sentencias
	insert = QSqlQuery ();
	insert.prepare(QString("INSERT INTO logger (TimeStamp,Type,Sender,Method,Message,File,Line,FullPath) VALUES (:timeStamp,:type,:sender,:method,:message,:file,:line,:fullpath)"));

	//creacion de la clase de visualizacion
	lg=new LoggerDlgControl(&baseDatos, this);    
        show();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	baseDatos.close();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}



/**
 * \brief Initialize table if not exists
*/
bool SpecificWorker::createTable(){
    QSqlQuery create;
    create.prepare(QString("CREATE TABLE logger (TimeStamp VARCHAR(20),Type VARCHAR(7), Sender VARCHAR(20),Method VARCHAR(12),Message VARCHAR(50),File VARCHAR(12),Line INTEGER,FullPath VARCHAR(100), PRIMARY KEY(TimeStamp,Sender DESC))"));
    return create.exec();
}
/**
 * \brief Insert a new row 
 * @param m Struct contains message information to be inserted
*/

void SpecificWorker::sendMessage(const RoboCompLogger::LogMessage &m)
{
    insert.bindValue(":timestamp",m.timeStamp.c_str());
    insert.bindValue(":type",m.type.c_str());
    insert.bindValue(":sender",m.sender.c_str());
    insert.bindValue(":method",m.method.c_str());
    insert.bindValue(":message",m.message.c_str());
    insert.bindValue(":file",m.file.c_str());
    insert.bindValue(":line",m.line);
    insert.bindValue(":fullpath",m.fullpath.c_str());
    //return insert.exec();
}







