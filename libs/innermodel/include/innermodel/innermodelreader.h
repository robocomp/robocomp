/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
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

#ifndef INNERMODELREADER_H
#define INNERMODELREADER_H

#include <QtXml/QtXml>

#include <innermodel/innermodel.h>
#include <innermodel/innermodelnode.h>
#include <innermodel/innermodelcamera.h>


class InnerModelReader
{
public:
	InnerModelReader();
	~InnerModelReader();
	static bool load(const QString &file, InnerModel *model);
	
private:
	static bool include(const QString &file, InnerModel *model, InnerModelNode *node);
	static void recursive(QDomNode parentDomNode, InnerModel *model, InnerModelNode *imNode);
	static QMap<QString, QStringList> getValidNodeAttributes();
};

#endif
