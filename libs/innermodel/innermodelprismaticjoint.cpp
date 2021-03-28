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

#include "innermodel/innermodelprismaticjoint.h"

InnerModelPrismaticJoint::InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),0,0,0,0,0,0, 0, parent_)
{
	#if FCL_SUPPORT==1
		collisionObject = NULL;
	#endif
	min = min_;
	max = max_;
	port = port_;
	axis = axis_;
	home = home_;
	offset = offset_;
	fixed = false;
	setPosition(val_);
}

void InnerModelPrismaticJoint::print(bool verbose)
{
	printf("Prismatic Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		((QMat *)this)->print(qPrintable(id));
		getTr().print(id+"_T");
	}
}

void InnerModelPrismaticJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}

void InnerModelPrismaticJoint::update()
{
	updateChildren();
}

float InnerModelPrismaticJoint::getPosition()
{
	return value;
}

float InnerModelPrismaticJoint::setPosition(float v)
{
	float ret;
	if (v <= max and v >= min)
	{
		ret = v;
	}
	else
	{
		if (v > max)
			ret = max;
		else
			ret = min;
	}
	value = v = ret;
	if (axis == "x")
	{
		set(0,0,0, v+offset,0,0);
	}
	else if (axis == "y")
	{
		set(0,0,0, 0,v+offset,0);
	}
	else if (axis == "z")
	{
		set(0,0,0, 0,0,v+offset);
	}
	else
	{
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
	}
	return ret;
}

InnerModelNode * InnerModelPrismaticJoint::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelPrismaticJoint *ret = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, (InnerModelTransform *) parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

