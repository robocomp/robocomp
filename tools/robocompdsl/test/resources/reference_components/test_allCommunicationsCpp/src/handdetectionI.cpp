/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "handdetectionI.h"

HandDetectionI::HandDetectionI(GenericWorker *_worker)
{
	worker = _worker;
}


HandDetectionI::~HandDetectionI()
{
}


int HandDetectionI::addNewHand(const int expectedHands, const RoboCompHandDetection::TRoi &roi, const Ice::Current&)
{
	return worker->HandDetection_addNewHand(expectedHands, roi);
}

RoboCompHandDetection::Hands HandDetectionI::getHands(const Ice::Current&)
{
	return worker->HandDetection_getHands();
}

int HandDetectionI::getHandsCount(const Ice::Current&)
{
	return worker->HandDetection_getHandsCount();
}

