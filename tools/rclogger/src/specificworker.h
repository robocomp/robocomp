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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <QSqlDatabase>
#include <QSqlQuery>
//#include "loggerdlgcontrol.h"
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QtGui>
#include <QtCore>
#include <QAction>
#include <QMenuBar>
#include <QWidget>
#include <QTimer>
#include <QSqlTableModel>
#include <QSqlQuery>
#include <QSqlIndex>
#include <QTableView>
#include <qsqlrecord.h>
#include <QSqlDatabase>
#include "ui_mainUI.h"

#define UPDATE_TIME 250
class QEnterAwareTable : public QTableView
{
Q_OBJECT
public:
	QEnterAwareTable(QWidget *p=NULL) : QTableView(p)
	{
		mouseInside = false;
	}
	bool inside()
	{
		return mouseInside;
	}
	void mousePressEvent(QMouseEvent *e){
	      emit pressMouse(QPoint(e->x(),e->y()));
	}
	void mouseDoubleClickEvent(QMouseEvent *e){
	      emit doubleClick(QPoint(e->x(),e->y()));
	}
protected:
	void enterEvent(QEvent *event)
	{
		mouseInside = true;
	}
	void leaveEvent(QEvent *event)
	{
		mouseInside = false;
	}
private:
	bool mouseInside;
signals:
	void pressMouse(QPoint p);
	void doubleClick(QPoint p);
		    
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void sendMessage(const LogMessage &m);
private:
	QMenuBar *mainMenu;
	QMenu *menuFile;
	QAction *actionExit;
	QSqlQuery senders,rowcount;
	QSqlTableModel *tModel;
	QTimer tUpdate;
	QEnterAwareTable *tView;
	QVector<QString>vfilter;	//0 time, 1 sender,2 type
	int currentRow;
	int typecount;
	bool updateWindow;
	bool lastRow;
	QString f;
	//popup
	QFormLayout *layout;
	QWidget *popup;
	QListWidget *listW;
	//
	void selectSenders();
	void filter();
	void resizeTable();
	void closeEvent(QCloseEvent *e);
	void resizeEvent ( QResizeEvent * event );
        
public slots:
	void compute();
        void update();  
	void timeFilter(QDateTime t);
	void senderFilter();
	void typeFilter();
	void updateRowCount();
	void on_pBsql_clicked();
	void on_pBlast_clicked();
	void on_pBplay_clicked();
	void on_pBfirst_clicked();
	void on_pBnext_clicked();
	void on_pBprevious_clicked();
	void mousePressEvent(QPoint p);
	void mouseDobleClickEvent(QPoint p);
	// void on_cBInfo_clicked();
	// void on_cBDebug_clicked();
	// void on_cBError_clicked();
	void showSelectionMode(QString s);
	//columns
	void on_colTimeStamp_clicked();
	void on_colType_clicked();
	void on_colSender_clicked();
	void on_colMethod_clicked();
	void on_colFile_clicked();
	void on_colLine_clicked();
	void forceExit();
        
private:
	QTimer timer;
	QSqlDatabase baseDatos;
	QSqlQuery insert;
	QString dbname;

	
	bool createTable();

	
};

#endif

