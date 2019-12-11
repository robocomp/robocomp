#include "qstateMachineWrapper.h"

QStateMachineWrapper::QStateMachineWrapper(QWidget *parent) : QStateMachine()
{
	widget = new GraphWidget(parent);
	widget->show();
	widget->resize(400,300);

	nkey = 0 ;

	tupdate.start(50);
	connect(&tupdate,SIGNAL(timeout()),this,SLOT(update()));
}
QStateMachineWrapper::~QStateMachineWrapper()
{
}

QState* QStateMachineWrapper::addState(QString id, ChildMode childMode, QState *parent)
{
	QState *state;
	if (parent != NULL)
	{
		//qDebug() <<"Wrapper: add state"<<id<<nkey<<"parent"<<parent->objectName();
		state = new QState(childMode, parent);
		state->setObjectName(QString::number(nkey));
		widget->addNode(id,nkey,parent->objectName().toInt() );
	}
	else
	{
		//qDebug() <<"Wrapper: add state"<<id<<nkey;
		state = new QState(childMode, this);
		state->setObjectName(QString::number(nkey));
		QStateMachine::addState(state);
		widget->addNode(id,nkey);
	}
	nodes_table[id] = nkey;
	nkey++;
	return state;
}

QFinalState* QStateMachineWrapper::addFinalState(QString id, QState *parent)
{
	QFinalState *state;
	if (parent != NULL)
	{
//		qDebug() <<"Wrapper: add final state"<<id<<nkey<<"parent"<<parent->objectName();
		state = new QFinalState(parent);
		state->setObjectName(QString::number(nkey) );
		widget->addNode(id,nkey,parent->objectName().toInt(),Final);
	}
	else
	{
// 		qDebug() <<"Wrapper: add final state"<<id<<nkey;
		state = new QFinalState(this);
		QStateMachine::addState(state);
		state->setObjectName(QString::number(nkey));
		widget->addNode(id,nkey,Final);
	}
	nodes_table[id] = nkey;
	nkey++;
	return state;
}

QSignalTransition* QStateMachineWrapper::addTransition(QAbstractState *source,QObject *sender,const char *signal,QAbstractState *target,EdgeType t)
{
	QSignalTransition *transition;
	transition = ((QState *)source)->addTransition(sender,signal,target);

	widget->addEdge(source->objectName().toInt(),target->objectName().toInt(),t);

	return transition;
}

void QStateMachineWrapper::update()
{
	QState *state;
	QList<int> activeNodes;
	activeNodes.clear();
	foreach(QAbstractState *item,this->configuration())
	{
		if ((state=dynamic_cast<QState *>(item)) != NULL)
			activeNodes.append(state->objectName().toInt());
	}
	widget->setActiveNodes(activeNodes);
}

void QStateMachineWrapper::setInitialState(QAbstractState *parent,QAbstractState *initial)
{

	Node *aux = widget->getNode(initial->objectName().toInt());

	if(aux != NULL)
	{
// 		qDebug()<<"set initial state"<<aux->id<<((QState *)parent)->objectName();
		aux->setType(Initial);
		((QState *)parent)->setInitialState(initial);
	}
	else
		qDebug()<<"Initial state does not exist";
}

void QStateMachineWrapper::setInitialState(QAbstractState *parent)
{
	Node *aux = widget->getNode(parent->objectName().toInt());
	if(aux != NULL)
	{
		aux->setType(Initial);
		QStateMachine::setInitialState(parent);
	}
}
