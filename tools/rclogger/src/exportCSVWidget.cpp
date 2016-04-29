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
#include "exportCSVWidget.h"

/**
* \brief Default constructor
*/
ExportCSVWidget::ExportCSVWidget(QSqlTableModel *tModel)
{
	setupUi(this);
	connect(buttonBox,SIGNAL(accepted()),this,SLOT(exportData())); 
	data = tModel;
	columnNames = "";
	selectedColumns.clear();
	
}
/**
* \brief Default destructor
*/
ExportCSVWidget::~ExportCSVWidget()
{

}

void ExportCSVWidget::prepareColumns()
{
	if (cb_timeStamp->isChecked()){
		selectedColumns.append(0);
		columnNames += "TimeStamp" + cb_separator->currentText();
	}
	if (cb_type->isChecked()){
		selectedColumns.append(1);
		columnNames += "Type" + cb_separator->currentText();
	}
	if (cb_sender->isChecked()){
		selectedColumns.append(2);
		columnNames += "Sender" + cb_separator->currentText();
	}
	if (cb_method->isChecked()){
		selectedColumns.append(3);
		columnNames += "Method" + cb_separator->currentText();
	}
	if (cb_message->isChecked()){
		selectedColumns.append(4);
		columnNames += "Message" + cb_separator->currentText();
	}
	if (cb_file->isChecked()){
		selectedColumns.append(5);
		columnNames += "File" + cb_separator->currentText();
	}
	if (cb_line->isChecked()){
		selectedColumns.append(6);
		columnNames += "Line" + cb_separator->currentText();
	}
	// remove last separator
	columnNames.chop(columnNames.size() - columnNames.lastIndexOf(cb_separator->currentText()));
}

void ExportCSVWidget::exportToFile(QString filename)
{
	ofstream file;
	file.open(filename.toUtf8().constData());
	//export columns names
	if (cb_names->isChecked())
	{
		file << columnNames.toUtf8().constData() << "\n";
	}
	//export data
	QString line;
	for(int i=0;i<data->rowCount();i++){
		line = "";
		for(int j=0;j<selectedColumns.size();j++){
			line += data->index(i,selectedColumns[j]).data().toString();
			if (j < selectedColumns.size()-1)
				line += cb_separator->currentText();
		}
		file << line.toUtf8().constData() << "\n";
	}
	file.close();
}

void ExportCSVWidget::exportData()
{
	QString file = le_file->text();
	if (not file.contains(".csv"))
		file += ".csv";
	qDebug()<<"save filename"<<file;
	prepareColumns();
	exportToFile(file);
	QMessageBox msgBox;
	msgBox.setText("Data exported.");
	msgBox.exec();
	
}