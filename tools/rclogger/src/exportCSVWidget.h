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

#ifndef EXPORTCSVWIDGET_H
#define EXPORTCSVWIDGET_H

#include <QtGui>
#include <QSqlTableModel>
#include <QtWidgets/QMessageBox>
#include <stdint.h>
#include <fstream>
#include <qlog/qlog.h>
#include <ui_exportCSV.h>

using namespace std;

class ExportCSVWidget : public QDialog, public Ui_exportCSV
{
Q_OBJECT
private:
	QSqlTableModel *data;
	QString columnNames;
	QList<int> selectedColumns;
	
public:
	ExportCSVWidget(QSqlTableModel *tModel);
	~ExportCSVWidget();
	void prepareColumns();	
	void exportToFile(QString filename);
public slots:
	void exportData();
	
};
#endif 
