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
#ifndef HANDDETECTION_H
#define HANDDETECTION_H

// Ice includes
#include <Ice/Ice.h>
#include <HandDetection.h>

#include <config.h>
#include "genericworker.h"


class HandDetectionI : public virtual RoboCompHandDetection::HandDetection
{
public:
	HandDetectionI(GenericWorker *_worker);
	~HandDetectionI();

	int addNewHand(const int expectedHands, const RoboCompHandDetection::TRoi &roi, const Ice::Current&);
	RoboCompHandDetection::Hands getHands(const Ice::Current&);
	int getHandsCount(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
