
#ifndef QSTATEMACHINEWRAPPER_H
#define QSTATEMACHINEWRAPPER_H

#include <QStateMachine>
#include <QtCore>
#include "graphwidget.h"
#include "edge.h"


class QStateMachineWrapper : public QStateMachine
{
Q_OBJECT
public:
	QStateMachineWrapper(QWidget *parent=0);
	~QStateMachineWrapper();
	void resize(QSize s) { widget->resize(s); }
	
	
	QState *addState(QString id, ChildMode childMode=QState::ExclusiveStates, QState *parent=0);
	QFinalState *addFinalState(QString id, QState *parent=0);
	
	QSignalTransition* addTransition(QAbstractState *source,QObject *sender,const char *signal,QAbstractState *target,EdgeType t = Normal);
	
	void setInitialState(QAbstractState *parent,QAbstractState *initial);
	void setInitialState(QAbstractState *parent);

private:
	GraphWidget *widget;
	QTimer tupdate;
	int nkey;
	QMap<QString,int> nodes_table;
	
public slots:
	void update();
	void hide() { widget->hide(); }
	void show() { widget->show(); }
	
};
#endif
