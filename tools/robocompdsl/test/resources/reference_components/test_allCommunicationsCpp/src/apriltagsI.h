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
#ifndef APRILTAGS_H
#define APRILTAGS_H

// Ice includes
#include <Ice/Ice.h>
#include <AprilTags.h>

#include <config.h>
#include "genericworker.h"


class AprilTagsI : public virtual RoboCompAprilTags::AprilTags
{
public:
	AprilTagsI(GenericWorker *_worker);
	~AprilTagsI();

	void newAprilTag(const RoboCompAprilTags::tagsList &tags, const Ice::Current&);
	void newAprilTagAndPose(const RoboCompAprilTags::tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
