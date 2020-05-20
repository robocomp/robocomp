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
	twoState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(twoState);
	threeState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(threeState);
	fourState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(fourState);
	oneState = new QState(QState::ExclusiveStates);
	myStateMachine.addState(oneState);
	fiveState = new QFinalState();
	myStateMachine.addState(fiveState);
	test2sub1State = new QState(QState::ExclusiveStates, twoState);
	myStateMachine.addState(test2sub1State);
	test2sub2State = new QState(QState::ExclusiveStates, twoState);
	myStateMachine.addState(test2sub2State);
	test2sub21State = new QState(QState::ExclusiveStates, test2sub2State);
	myStateMachine.addState(test2sub2State);
	test2sub22State = new QFinalState(test2sub2State);
	test3sub1State = new QState(QState::ExclusiveStates, threeState);
	myStateMachine.addState(test3sub1State);
	test3sub2State = new QState(QState::ExclusiveStates, threeState);
	myStateMachine.addState(test3sub2State);
	test3sub3State = new QState(QState::ExclusiveStates, threeState);
	myStateMachine.addState(test3sub3State);
	test4sub2State = new QState(QState::ExclusiveStates, fourState);
	myStateMachine.addState(test4sub2State);
	test4sub1State = new QState(QState::ExclusiveStates, fourState);
	myStateMachine.addState(test4sub2State);

	myStateMachine.setInitialState(oneState);
	test2sub2State->setInitialState(test2sub21State);
	fourState->setInitialState(test4sub1State);

	oneState->addTransition(this, SIGNAL(t_one_to_two()), twoState);
	twoState->addTransition(this, SIGNAL(t_two_to_three()), threeState);
	threeState->addTransition(this, SIGNAL(t_three_to_four()), fourState);
	fourState->addTransition(this, SIGNAL(t_four_to_one()), oneState);
	fourState->addTransition(this, SIGNAL(t_four_to_five()), fiveState);
	test2sub1State->addTransition(this, SIGNAL(t_test2sub1_to_test2sub2()), test2sub2State);
	test2sub2State->addTransition(this, SIGNAL(t_test2sub2_to_test2sub2()), test2sub2State);
	test2sub21State->addTransition(this, SIGNAL(t_test2sub21_to_test2sub21()), test2sub21State);
	test2sub21State->addTransition(this, SIGNAL(t_test2sub21_to_test2sub22()), test2sub22State);
	test3sub1State->addTransition(this, SIGNAL(t_test3sub1_to_test3sub1()), test3sub1State);
	test3sub2State->addTransition(this, SIGNAL(t_test3sub2_to_test3sub2()), test3sub2State);
	test4sub1State->addTransition(this, SIGNAL(t_test4sub1_to_test4sub2()), test4sub2State);
	test4sub2State->addTransition(this, SIGNAL(t_test4sub2_to_test4sub1()), test4sub1State);

	QObject::connect(twoState, SIGNAL(entered()), this, SLOT(sm_two()));
	QObject::connect(threeState, SIGNAL(entered()), this, SLOT(sm_three()));
	QObject::connect(fourState, SIGNAL(entered()), this, SLOT(sm_four()));
	QObject::connect(oneState, SIGNAL(entered()), this, SLOT(sm_one()));
	QObject::connect(fiveState, SIGNAL(entered()), this, SLOT(sm_five()));
	QObject::connect(test2sub1State, SIGNAL(entered()), this, SLOT(sm_test2sub1()));
	QObject::connect(test2sub2State, SIGNAL(entered()), this, SLOT(sm_test2sub2()));
	QObject::connect(test2sub21State, SIGNAL(entered()), this, SLOT(sm_test2sub21()));
	QObject::connect(test2sub22State, SIGNAL(entered()), this, SLOT(sm_test2sub22()));
	QObject::connect(test3sub1State, SIGNAL(entered()), this, SLOT(sm_test3sub1()));
	QObject::connect(test3sub2State, SIGNAL(entered()), this, SLOT(sm_test3sub2()));
	QObject::connect(test3sub3State, SIGNAL(entered()), this, SLOT(sm_test3sub3()));
	QObject::connect(test4sub1State, SIGNAL(entered()), this, SLOT(sm_test4sub1()));
	QObject::connect(test4sub2State, SIGNAL(entered()), this, SLOT(sm_test4sub2()));

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
