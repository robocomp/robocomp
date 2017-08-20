/*
 *    Copyright (C)2017 by YOUR NAME HERE
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
GenericWorker::GenericWorker(MapPrx& mprx) :
QObject()
{

	mutex = new QMutex(QMutex::Recursive);

	Period = BASIC_PERIOD;
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
// 	timer.start(Period);
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


void GenericWorker::substract(const int num1,const int num2,int &result)
{
	uint cid = substractBuffer.push(std::make_tuple(num1,num2));
	while(!substractBuffer.isFinished(cid));
	std::tie(result) = substractBuffer.result(cid);
}

void GenericWorker::printmsg(const string &message)
{
	uint cid = printmsgBuffer.push(std::make_tuple(message));
}

int GenericWorker::sum(const int num1,const int num2)
{
	uint cid = sumBuffer.push(std::make_tuple(num1,num2));
	while(!sumBuffer.isFinished(cid));
	int ret;
	std::tie(ret) = sumBuffer.result(cid);
	return ret;
}

int GenericWorker::divide(const int divident,const int divisor,int &reminder)
{
	uint cid = divideBuffer.push(std::make_tuple(divident,divisor));
	while(!divideBuffer.isFinished(cid));
	int ret;
	std::tie(ret,reminder) = divideBuffer.result(cid);
	return ret;
}




