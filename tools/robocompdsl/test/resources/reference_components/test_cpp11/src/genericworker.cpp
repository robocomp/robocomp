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
GenericWorker::GenericWorker(TuplePrx tprx) : QObject()
{

	//Initialization State machine
	computeState = new QState(QState::ExclusiveStates);
	defaultMachine.addState(computeState);
	initializeState = new QState(QState::ExclusiveStates);
	defaultMachine.addState(initializeState);
	finalizeState = new QFinalState();
	defaultMachine.addState(finalizeState);

	defaultMachine.setInitialState(initializeState);

	initializeState->addTransition(this, SIGNAL(t_initialize_to_compute()), computeState);
	computeState->addTransition(this, SIGNAL(t_compute_to_compute()), computeState);
	computeState->addTransition(this, SIGNAL(t_compute_to_finalize()), finalizeState);

	QObject::connect(computeState, SIGNAL(entered()), this, SLOT(sm_compute()));
	QObject::connect(initializeState, SIGNAL(entered()), this, SLOT(sm_initialize()));
	QObject::connect(finalizeState, SIGNAL(entered()), this, SLOT(sm_finalize()));
	QObject::connect(&timer, SIGNAL(timeout()), this, SIGNAL(t_compute_to_compute()));

	//------------------
	omnirobot_proxy = std::get<0>(tprx);

	mutex = new QMutex(QMutex::Recursive);

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
