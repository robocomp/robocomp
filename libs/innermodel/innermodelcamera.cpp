/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "innermodelcamera.h"
#include <innermodel/innermodel.h>

InnerModelCamera::InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModel *innermodel_, InnerModelNode *parent_) 
: InnerModelNode(id_, parent_), innermodel(innermodel_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	camera = Cam(focal_, focal_, width_/2., height_/2.);
	camera.setSize(width, height);
	width = width_;
	height = height_;
	focal = focal_;
}

void InnerModelCamera::print(bool verbose)
{
	if (verbose) camera.print(QString("Camera: ")+id);
}

void InnerModelCamera::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<camera id=\"" << id << "\" width=\"" << QString::number(width, 'g', 10) << "\" height=\"" << QString::number(height, 'g', 10) << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
}

void InnerModelCamera::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

InnerModelNode * InnerModelCamera::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelCamera *ret = new InnerModelCamera(id, width, height, focal, innermodel, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	ret->camera = camera;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

QVec InnerModelCamera::project(QString reference, QVec origVec, QString cameraId)
{
	origVec = innermodel->transform(cameraId, origVec, reference);
	QVec pc;

	pc = camera.project(origVec);

	return QVec::vec3(pc(0), pc(1), origVec.norm2());
}
