/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#include "outtestI.h"

outTestI::outTestI(GenericWorker *_worker)
{
	worker = _worker;
}


outTestI::~outTestI()
{
}

void outTestI::substract(const int  num1, const int  num2,  int  &result, const Ice::Current&)
{
	worker->substract(num1, num2, result);
}

void outTestI::printmsg(const string  &message, const Ice::Current&)
{
	worker->printmsg(message);
}

int outTestI::sum(const int  num1, const int  num2, const Ice::Current&)
{
	return worker->sum(num1, num2);
}

int outTestI::divide(const int  divident, const int  divisor,  int  &reminder, const Ice::Current&)
{
	return worker->divide(divident, divisor, reminder);
}

