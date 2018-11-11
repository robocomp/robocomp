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
#include "specificworker.h"


DisplayI::DisplayI(std::shared_ptr<SpecificWorker> _worker)
{
	worker = _worker;
}


DisplayI::~DisplayI()
{
}

void DisplayI::add(QString id_)
{
	id=id_;
}

void DisplayI::remove(QString id)
{
}

void DisplayI::setImageFromFile(const string  &pathImg, const Ice::Current&)
{
	guard gl(worker->innerModel->mutex);
	QString m = "RoboCompDisplay::setImage()";
	InnerModelDisplay *aux = dynamic_cast<InnerModelDisplay*>(worker->getNode(id, m));
	osg::Image *image = osgDB::readImageFile(pathImg);
	aux->texture = QString::fromStdString(pathImg);
	if (image == nullptr)
	{
		//qDebug() << __LINE__ << "Couldn't load texture:" << texture.c_str();
		throw "SetImage::Couldn't load image" + pathImg;
	}
	worker->imv->planesHash[aux->id]->setImage(image);
}

void DisplayI::setImage(const RoboCompDisplay::Image  &img, const Ice::Current&)
{
	guard gl(worker->innerModel->mutex);
	QImage im = QImage(&img.Img[0],img.width,img.height, QImage::Format_ARGB32);
	im.save("/var/tmp/tmp.jpg");	
	QString m = "RoboCompDisplay::setImage()";
	InnerModelDisplay *aux = dynamic_cast<InnerModelDisplay*>(worker->getNode(id, m));
	aux->texture = QString("/var/tmp/tmp.jpg");
	osg::Image *image = osgDB::readImageFile("/var/tmp/tmp.jpg");
	if (image == nullptr)
	{
		//qDebug() << __LINE__ << "Couldn't load texture:" << texture.c_str();
		throw "SetImage::Couldn't load image";
	}
	worker->imv->planesHash[aux->id]->setImage(image);
}
