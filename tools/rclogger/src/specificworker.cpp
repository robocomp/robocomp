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
        setupUi(this);
        
	baseDatos = QSqlDatabase::addDatabase("QSQLITE");
	baseDatos.setDatabaseName("bd.db");
	if (!baseDatos.open())
		qDebug()<<"Database could not be openned";

	
	QStringList ltables = baseDatos.tables();
	if (!ltables.contains("logger"))
		qDebug()<<"Create table:"<< createTable();

	
	//preparacion de las sentencias
	insert = QSqlQuery();
	insert.prepare(QString("INSERT INTO logger (TimeStamp,Type,Sender,Method,Message,File,Line,FullPath) VALUES (:timeStamp,:type,:sender,:method,:message,:file,:line,:fullpath)"));

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
	tView->setSortingEnabled(true);
	tView->horizontalHeader()->setMovable(true);
	tView->setModel(tModel);
	QFont f=QApplication::font();
	f.setPointSize(0.8*f.pointSizeF());
	tView->setFont(f);
	tView->setColumnHidden(7,true);
	//icons
/*	pBnext->setIcon(QIcon(QPixmap("//opt//robocomp//share//rclogger//icons//next.png")));
	pBlast->setIcon(QIcon(QPixmap("/opt/robocomp/share/rclogger/icons/last.png")));
	pBprevious->setIcon(QIcon(QPixmap("/opt/robocomp/share/rclogger/icons/previous.png")));
	pBfirst->setIcon(QIcon(QPixmap("/opt/robocomp/share/rclogger/icons/first.png")));
	pBplay->setIcon(QIcon(QPixmap("/opt/robocomp/share/rclogger/icons/pause.png")));*/
        pBnext->setText(">");
        pBlast->setText(">>");
        pBprevious->setText("<");
        pBfirst->setText("<<");
        pBplay->setText("Pause");
        
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
	//ComboBox
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
	rowcount = QSqlQuery(baseDatos);
	rowcount.prepare("SELECT max(rowid) FROM logger;");
	senders = QSqlQuery(baseDatos);
	senders.prepare("SELECT DISTINCT Sender FROM logger;");
	//Main menu
	mainMenu =new QMenuBar(this);
	mainMenu->setFixedWidth(300);
	menuFile = mainMenu->addMenu("File");
	actionExit = menuFile->addAction("Exit");


	connect(actionExit,SIGNAL(triggered(bool)),this,SLOT(forceExit()));
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
	usleep(100000);
        
}



/**
 * \brief Initialize table if not exists
*/
bool SpecificWorker::createTable()
{
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
//	printf("%s: %s(%d): %s\n", m.sender.c_str(), m.file.c_str(), m.line, m.message.c_str());
	insert.bindValue(":timestamp",m.timeStamp.c_str());
	insert.bindValue(":type",m.type.c_str());
	insert.bindValue(":sender",m.sender.c_str());
	insert.bindValue(":method",m.method.c_str());
	insert.bindValue(":message",m.message.c_str());
	insert.bindValue(":file",m.file.c_str());
	insert.bindValue(":line",m.line);
	insert.bindValue(":fullpath",m.fullpath.c_str());
        insert.exec();
}

void SpecificWorker::selectSenders()
{
	QListWidgetItem *n;
	senders.exec();
	bool sig=senders.next();
	while (sig)
	{
 		int i=0;
                while (i < sendersWidget->count() and sendersWidget->item(i)->text()!=senders.value(0).toString())
		{
			i++;
		}
		if (i >= sendersWidget->count())
		{
			n=new QListWidgetItem(senders.value(0).toString());
			sendersWidget->addItem(n);
			n->setSelected(true);
                        qDebug()<<"Sender added:"<<senders.value(0).toString();
		}
		sig = senders.next();
	}
}

void SpecificWorker::update()
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

void SpecificWorker::filter()
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

void SpecificWorker::timeFilter(QDateTime t)
{
	vfilter[0] = QString("TimeStamp >= \""+t.toString("yyyy.MM.dd hh:mm:ss:zzz")+"\"");
	filter();
}

void SpecificWorker::senderFilter()
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

void SpecificWorker::typeFilter()
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

void SpecificWorker::showSelectionMode(QString s)
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

void SpecificWorker::on_pBsql_clicked()
{
	vfilter[0]=sqlLine->text();
	filter();
}

void SpecificWorker::updateRowCount()
{
	rowcount.exec();
	rowcount.next();
	currentRow = rowcount.value(0).toInt();
}

void SpecificWorker::on_pBlast_clicked()
{
	lastRow=true;
}

void SpecificWorker::on_pBplay_clicked()
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

void SpecificWorker::on_pBfirst_clicked()
{
	currentRow=0;
	lastRow=false;
}

void SpecificWorker::on_pBnext_clicked()
{
	currentRow+=10;
	lastRow=false;
	if (currentRow > tModel->rowCount()-1)
	{
		currentRow = tModel->rowCount()-1;
	}
}

void SpecificWorker::on_pBprevious_clicked()
{
	currentRow-=10;
	lastRow=false;
	if (currentRow < 0)
	{
		currentRow = 0;
	}
}

void SpecificWorker::mousePressEvent(QPoint p)
{
	lastRow=false;
	currentRow=tView->rowAt(p.y());
}

void SpecificWorker::mouseDobleClickEvent(QPoint p)
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

void SpecificWorker::closeEvent(QCloseEvent *e)
{
	senders.clear();
	delete tModel;
}

void SpecificWorker::resizeEvent ( QResizeEvent * event )
{
      tView->resize(table->size()); 
      resizeTable();
}

void SpecificWorker::resizeTable()
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

void SpecificWorker::on_colTimeStamp_clicked()
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

void SpecificWorker::on_colType_clicked()
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

void SpecificWorker::on_colSender_clicked()
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

void SpecificWorker::on_colMethod_clicked()
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

void SpecificWorker::on_colFile_clicked()
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

void SpecificWorker::on_colLine_clicked()
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

void SpecificWorker::forceExit()
{
    close();
}





