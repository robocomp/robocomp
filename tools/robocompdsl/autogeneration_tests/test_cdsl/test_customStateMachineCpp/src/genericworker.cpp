/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(MapPrx& mprx) : Ui_guiDlg()
{

	//Initialization State machine
	oneState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(oneState);
	twoState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(twoState);
	threeState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(threeState);
	fourState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(fourState);
	fiveState = new QFinalState();
	myStateMachine.addState(fiveState);

	myStateMachine.setInitialState(oneState);

	oneState->addTransition(this, SIGNAL(t_one_to_two()), twoState);
	twoState->addTransition(this, SIGNAL(t_two_to_three()), threeState);
	threeState->addTransition(this, SIGNAL(t_three_to_four()), fourState);
	fourState->addTransition(this, SIGNAL(t_four_to_one()), oneState);
	fourState->addTransition(this, SIGNAL(t_four_to_five()), fiveState);

	QObject::connect(oneState, SIGNAL(entered()), this, SLOT(sm_one()));
	QObject::connect(twoState, SIGNAL(entered()), this, SLOT(sm_two()));
	QObject::connect(threeState, SIGNAL(entered()), this, SLOT(sm_three()));
	QObject::connect(fourState, SIGNAL(entered()), this, SLOT(sm_four()));
	QObject::connect(fiveState, SIGNAL(entered()), this, SLOT(sm_five()));

	//------------------

	mutex = new QMutex(QMutex::Recursive);


	#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif
	Period = BASIC_PERIOD;

}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}
