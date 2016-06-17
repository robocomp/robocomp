/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "loggerdlgcontrol.h"



LoggerDlgControl::LoggerDlgControl(QSqlDatabase *_bD, QWidget* parent) : QWidget(parent) , Ui::guiDlg() , bD(_bD)
{
	setupUi(this);
	//tModel
	tModel= new QSqlTableModel();
	tModel->setTable("logger");
	tModel->setSort(0,Qt::AscendingOrder);
	tModel->select();
	//tView
	tView = new QEnterAwareTable(table);
	tView->setCornerButtonEnabled(false);
	tView->setSelectionBehavior(QAbstractItemView::SelectRows);
	tView->setEditTriggers(QAbstractItemView::NoEditTriggers);//non editable
	tView-> setSortingEnabled(true);
	tView->horizontalHeader()->setMovable(true);
	tView->setModel(tModel);
	QFont f=QApplication::font();
	f.setPointSize(0.8*f.pointSizeF());
	tView->setFont(f);
	tView->setColumnHidden(7,true);
	//icons
	pBnext->setIcon(QIcon(QPixmap("../icons/next.png")));
	pBlast->setIcon(QIcon(QPixmap("../icons/last.png")));
	pBprevious->setIcon(QIcon(QPixmap("../icons/previous.png")));
	pBfirst->setIcon(QIcon(QPixmap("../icons/first.png")));
	pBplay->setIcon(QIcon(QPixmap("../icons/pause.png")));

	timeStampEdit->setMinimumDateTime(tModel->record(0).value("TimeStamp").toDateTime());

	tUpdate.start(UPDATE_TIME);
	connect(&tUpdate, SIGNAL(timeout()), this, SLOT(update()));
	connect(timeStampEdit,SIGNAL(dateTimeChanged(QDateTime)),this,SLOT(timeFilter(QDateTime)));
	connect(sendersWidget,SIGNAL(itemSelectionChanged ()),this,SLOT(senderFilter()));
	connect(tView,SIGNAL(pressMouse(QPoint)),this,SLOT(mousePressEvent(QPoint)));
	connect(tView,SIGNAL(doubleClick(QPoint)),this,SLOT(mouseDobleClickEvent(QPoint)));
	connect(selectionBox,SIGNAL(activated(QString)),this,SLOT(showSelectionMode(QString)));
	connect(cBDebug,SIGNAL(clicked(bool)),this,SLOT(typeFilter()));
	connect(cBError,SIGNAL(clicked(bool)),this,SLOT(typeFilter()));
	connect(cBInfo,SIGNAL(clicked(bool)),this,SLOT(typeFilter()));

	vfilter.resize(3);
	vfilter.fill("");
	currentRow=tModel->rowCount()-1;
	lastRow=true;
	updateWindow=true;
	//ComoBox
	selectionBox->addItem("NONE");
	selectionBox->addItem("SQL");
	selectionBox->addItem("USER");
	showSelectionMode("NONE");
	//popup window
	layout =new QFormLayout();
	popup = new QWidget();
	popup->setLayout(layout);
	popup->setWindowTitle("Row content");
	popup->resize(600, 250);
	listW=new QListWidget(popup);
	layout->addRow(listW);
	listW->show();
	resizeTable();
	//rowCount
	rowcount = QSqlQuery(*bD);
	rowcount.prepare("SELECT max(rowid) FROM logger;");
	senders = QSqlQuery(*bD);
	senders.prepare("SELECT DISTINCT Sender FROM logger;");
	//Main menu
	mainMenu =new QMenuBar(this);
	mainMenu->setFixedWidth(300);
	menuFile = mainMenu->addMenu("File");
	actionExit = menuFile->addAction("Exit");


	printf("%s: %d\n", __FILE__, __LINE__);
	connect(actionExit,SIGNAL(triggered(bool)),this,SLOT(forceExit()));

	printf("%s: %d\n", __FILE__, __LINE__);
	mainMenu->show();
	printf("%s: %d\n", __FILE__, __LINE__);
}

LoggerDlgControl::~LoggerDlgControl()
{

}

void LoggerDlgControl::selectSenders()
{
	QListWidgetItem *n;
	senders.exec();
	bool sig=senders.next();
	while (sig)
	{
printf("%s: %d\n", __FILE__, __LINE__);
 		int i=0;
 		while (i < sendersWidget->count() and sendersWidget->item(i)->text()!=senders.value(0).toString())
		{
printf("%s: %d\n", __FILE__, __LINE__);
			i++;
		}
		if (i >= sendersWidget->count())
		{
			n=new QListWidgetItem(senders.value(0).toString());
			sendersWidget->addItem(n);
			n->setSelected(true);
		}
		sig = senders.next();
printf("%s: %d\n", __FILE__, __LINE__);
	}
}

void LoggerDlgControl::update()
{
	selectSenders();
	if (updateWindow)
	{
		if (not tView->inside())
		{
			tModel->select();
			table->setBackgroundRole(QPalette::Window);
		}
		else
		{
			table->setBackgroundRole(QPalette::Highlight);
		}
 		if (lastRow)
		{
			updateRowCount();
		}
		tView->selectRow(currentRow);
	}
}

void LoggerDlgControl::filter()
{
	f="";
	for (int i=0;i<vfilter.size();i++)
	{
		if (vfilter[i]!="")
		{
			if (f!="")
			{
				f+=" and ("+vfilter[i]+")";
			}
			else
			{
				f="("+vfilter[i]+")";
			}
		}
	}
	tModel->setFilter(f);
	resizeTable();
}

void LoggerDlgControl::timeFilter(QDateTime t)
{
	vfilter[0] = QString("TimeStamp >= \""+t.toString("yyyy.MM.dd hh:mm:ss:zzz")+"\"");
	filter();
}

void LoggerDlgControl::senderFilter()
{
	QList<QListWidgetItem *> list=sendersWidget->selectedItems ();
	QString sFilter="";
	for (int i=0; i<list.size(); i++)
	{
		if (sFilter!="")
		{
			sFilter+=" or ";
		}
		sFilter+="Sender = \""+list[i]->text()+"\"";		
	}
	vfilter[1] = sFilter;
	filter();
}

void LoggerDlgControl::typeFilter()
{
	QString tFilter="";
	if (cBInfo->checkState()==Qt::Checked)
	{
		tFilter="Type = \"Info\"";
	}
	if (cBError->checkState()==Qt::Checked)
	{
		if (tFilter!="")
		{
			tFilter+=" or ";
		}
		tFilter+="Type = \"Error\"";
	}
	if (cBDebug->checkState()==Qt::Checked)
	{
		if (tFilter!="")
		{
			tFilter+=" or ";
		}
		tFilter+="Type = \"Debug\"";
	}
	if (tFilter=="")
	{
		tFilter+="Type = \"NONE\"";
	}
	vfilter[2]=tFilter;
	filter();
}

void LoggerDlgControl::showSelectionMode(QString s)
{
	if (s=="NONE")
	{
		predefSel->setVisible(false);
		userSel->setVisible(false);
	}
	else
	{
		vfilter.fill("");
		if (s=="SQL")
		{
			userSel->setVisible(true);
			if (predefSel->isVisible())
			{
				predefSel->setVisible(false);
			}
		}
		else
		{
			if (userSel->isVisible())
			{
				userSel->setVisible(false);
			}
			predefSel->setVisible(true);
			timeFilter(timeStampEdit->dateTime());
			typeFilter();
			senderFilter();
		}
	}
	QSize a = size();
	resize(a.width()+1, a.height()+1); 
	resize(a.width(), a.height());
	repaint();
	resizeTable();
}

void LoggerDlgControl::on_pBsql_clicked()
{
	vfilter[0]=sqlLine->text();
	filter();
}

void LoggerDlgControl::updateRowCount()
{
	rowcount.exec();
	rowcount.next();
	currentRow = rowcount.value(0).toInt();
}

void LoggerDlgControl::on_pBlast_clicked()
{
	lastRow=true;
}

void LoggerDlgControl::on_pBplay_clicked()
{
	if (pBplay->isChecked())
	{
		updateWindow=true;
		pBplay->setIcon(QIcon(QPixmap("../icons/pause.png")));
	}
	else
	{
		updateWindow=false;
		pBplay->setIcon(QIcon(QPixmap("../icons/play.png")));
	}
}

void LoggerDlgControl::on_pBfirst_clicked()
{
	currentRow=0;
	lastRow=false;
}

void LoggerDlgControl::on_pBnext_clicked()
{
	currentRow+=10;
	lastRow=false;
	if (currentRow > tModel->rowCount()-1)
	{
		currentRow = tModel->rowCount()-1;
	}
}

void LoggerDlgControl::on_pBprevious_clicked()
{
	currentRow-=10;
	lastRow=false;
	if (currentRow < 0)
	{
		currentRow = 0;
	}
}

void LoggerDlgControl::mousePressEvent(QPoint p)
{
	lastRow=false;
	currentRow=tView->rowAt(p.y());	
}

void LoggerDlgControl::mouseDobleClickEvent(QPoint p)
{
	listW->clear();
	QString string;
	QSqlRecord record=tModel->record(tView->rowAt(p.y()));
	for (int i=0;i<record.count();i++)
	{
		string=record.fieldName(i)+" => "+record.value(i).toString();
		listW->addItem(string);
	}
	popup->show();
}

void LoggerDlgControl::closeEvent(QCloseEvent *e)
{
	senders.clear();
	delete tModel;
}

void LoggerDlgControl::resizeEvent ( QResizeEvent * event )
{
      tView->resize(table->size()); 
      resizeTable();
}

void LoggerDlgControl::resizeTable()
{
	tView->resizeColumnsToContents();
	int availableWidth = tView->viewport()->width();
	for ( int i = 0; i < tView->horizontalHeader()->count(); i++ )
	{
		if ( i != 4 )
		{
			availableWidth -= tView->columnWidth(i);
		}
	}
	if ( availableWidth > 0 )
	{
		tView->setColumnWidth(4, availableWidth);
	}
}

void LoggerDlgControl::on_colTimeStamp_clicked()
{
	if (colTimeStamp->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(0,false);
	}
	else
	{
		tView->setColumnHidden(0,true);
	}
	resizeTable();
}

void LoggerDlgControl::on_colType_clicked()
{
	if (colType->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(1,false);
	}
	else
	{
		tView->setColumnHidden(1,true);
	}
	resizeTable();
}

void LoggerDlgControl::on_colSender_clicked()
{
	if (colSender->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(2,false);
	}
	else
	{
		tView->setColumnHidden(2,true);
	}
	resizeTable();
}

void LoggerDlgControl::on_colMethod_clicked()
{
	if (colMethod->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(3,false);
	}
	else
	{
		tView->setColumnHidden(3,true);
	}
	resizeTable();
}

void LoggerDlgControl::on_colFile_clicked()
{
	if (colFile->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(5,false);
	}
	else
	{
		tView->setColumnHidden(5,true);
	}
	resizeTable();
}

void LoggerDlgControl::on_colLine_clicked()
{
	if (colLine->checkState()==Qt::Checked)
	{
		tView->setColumnHidden(6,false);
	}
	else
	{
		tView->setColumnHidden(6,true);
	}
	resizeTable();
}

void LoggerDlgControl::forceExit()
{
    close();
}
