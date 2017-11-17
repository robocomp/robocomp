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
#include "displayI.h"

DisplayI::DisplayI(SpecificWorker *_worker, QObject *parent)
{
	worker = _worker;
}


DisplayI::~DisplayI()
{
}

void JointMotorI::add(QString id)
{
	id=id;
}

void JointMotorI::remove(QString id)
{
}

void DisplayI::setImage(const Image  &img, const Ice::Current&)
{
	QImage img =QImage(img.Img,img.width,img.height,QImage::Format_RGB888);
	img.save("tmp.jpg")
	worker->imm_setPlaneTexture(id, item, "tmp.jpg");
}
