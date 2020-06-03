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
#include "apriltagsI.h"

AprilTagsI::AprilTagsI(GenericWorker *_worker)
{
	worker = _worker;
}


AprilTagsI::~AprilTagsI()
{
}


void AprilTagsI::newAprilTag(const RoboCompAprilTags::tagsList &tags, const Ice::Current&)
{
	worker->AprilTags_newAprilTag(tags);
}

void AprilTagsI::newAprilTagAndPose(const RoboCompAprilTags::tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState, const Ice::Current&)
{
	worker->AprilTags_newAprilTagAndPose(tags, bState, hState);
}

