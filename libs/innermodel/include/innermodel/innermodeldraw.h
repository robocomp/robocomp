/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef INNERMODELDRAW_H
#define INNERMODELDRAW_H

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include <innermodel/innermodeltransform.h>
#include <innermodel/innermodeljoint.h>
#include <innermodel/innermodelplane.h>
#include <innermodel/innermodelmesh.h>
#include <innermodel/innermodelcamera.h>
#include <innermodel/innermodelpointcloud.h>

using namespace RMat;

class InnerModelDraw
{
public:
	InnerModelDraw(){};
	static void addMesh_ignoreExisting(InnerModelViewer *innerViewer, QString a, QString parent, QVec t, QVec r, QString path, QVec scale);
	static bool addTransform(InnerModelViewer *innerViewer, QString a, QString b/*, const RoboCompInnerModelManager::Pose3D & m*/);
	static bool addTransform_ignoreExisting(InnerModelViewer *innerViewer, QString a, QString b/*, const RoboCompInnerModelManager::Pose3D & m*/);
 /**
 * @brief Draws a segment parallel to the floor defined with a normalVector and the position of its center
 * 
 * @param innerViewer ...
 * @param name ...
 * @param parent ...
 * @param normalVector Vector normal to the segment
 * @param center coordinates of the middle point of the segment
 * @param length length of the segment
 * @param width the other two dimensions of the 3D prims forming the segment
 * @param texture color defined as the string: #rrggbb
 * @return void
 */
	static void drawLine(InnerModelViewer *innerViewer, QString name, QString parent, const QVec &normalVector, const QVec &center, float length, float width, QString texture = "#550000");
 /**
 * @brief Draws a line parallel the floor between point p1 and p2 wrt to the parent coordinate system
 * 
 * @param innerViewer ...
 * @param name ...
 * @param parent ...
 * @param p1 ...
 * @param p2 ...
 * @param width ...
 * @param texture ...
 * @return void
 */
	
	static void drawLine2Points(InnerModelViewer *innerViewer, QString name, QString parent, const QVec& p1, const QVec& p2, float width, QString texture);
	static bool removeObject(InnerModelViewer *innerViewer, QString name);
	static bool removeNode(InnerModelViewer *innerViewer, const QString &item);
	static bool addPlane_ignoreExisting(InnerModelViewer *innerViewer, const QString &a, const QString &b, const QVec &p, const QVec &n, const QString &texture, const QVec &size);
	static bool addPlane_notExisting(   InnerModelViewer *innerViewer, const QString &a, const QString &b, const QVec &p, const QVec &n, const QString &texture, const QVec &size);
	static bool setScale(InnerModelViewer *innerViewer, const QString item, float scaleX, float scaleY, float scaleZ);
	static bool setPlaneTexture(InnerModelViewer *innerViewer, const QString item, QString texture);
	static bool addJoint(InnerModelViewer* innerViewer, const QString item, const QString base, QVec t, QVec r, QString axis);

};

#endif // INNERMODELDRAW_H
