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

#include "innermodel/innermodel.h"
#include "innermodel/innermodeljoint.h"

class InnerModel;

InnerModelJoint::InnerModelJoint() : InnerModelTransform("invalid",QString("static"), 0,0,0, 0,0,0, 0, NULL)
{
	throw std::string("Can't actually build InnerModelJoint using the default constructor");
}

InnerModelJoint::InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
	
	#if FCL_SUPPORT==1
	collisionObject = NULL;
	#endif

		min = min_;
	max = max_;
	home = home_;
	port = port_;
	axis = axis_;
	if (axis == "x")
	{
		backl = lx_;
		backh = hx_;
		update(min, 0, 0, max, 0, 0);
	}
	else if (axis == "y")
	{
		backl = ly_;
		backh = hy_;
		update(0, min, 0, 0, max, 0);
	}
	else if (axis == "z")
	{
		backl = lz_;
		backh = hz_;
		update(0, 0, min, 0, 0, max);
	}
	else
	{
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
	}
}

void InnerModelJoint::print(bool verbose)
{
	printf("Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		((QMat *)this)->print(qPrintable(id));
		getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}

void InnerModelJoint::save(QTextStream &out, int tabs)
{
	QList<InnerModelNode*>::iterator c;
	for (int i=0; i<tabs; i++)
		out << "\t";

	out << "<joint id=\"" << id << "\" port=\"" << port << "\" axis=\"" <<QString::fromStdString(axis);

	out << "\" home=\"" << QString::number(home, 'g', 10);
	out << "\" min=\"" << QString::number(min, 'g', 10) << "\" max=\"" << QString::number(max, 'g', 10);
	out << "\" tx=\"" << QString::number(backtX, 'g', 10) << "\" ty=\"" << QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10);
	out << "\" rx=\"" << QString::number(backrX, 'g', 10) << "\" ry=\"" << QString::number(backrY, 'g', 10) << "\" rz=\"" << QString::number(backrZ, 'g', 10);

	out << "\">\n";

	for (c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);

	for (int i=0; i<tabs; i++) out << "\t";
	out << "</joint>\n";
}


void InnerModelJoint::update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_)
{
	if (axis == "x")
	{
		backh = hx_;
		backl = lx_;
	}
	else if (axis == "y")
	{
		backh = hy_;
		backl = ly_;
	}
	else if (axis == "z")
	{
		backh = hz_;
		backl = lz_;
	}
	fixed = true;
}

float InnerModelJoint::getAngle()
{
	if (axis == "x")
	{
		return backrX;
	}
	else if (axis == "y")
	{
		return backrY;
	}
	else
	{
		return backrZ;
	}
}

float InnerModelJoint::setAngle(float angle, bool force)
{
	float ret = angle;
	if (angle > max)
	{
		ret = max;
	}
	else if (angle < min)
	{
		ret = min;
	}

	if (axis == "x")
	{
		backrX = ret;
		set(ret,0,0, 0,0,0);
	}
	else if (axis == "y")
	{
		backrY = ret;
		set(0,ret,0, 0,0,0);
	}
	else if (axis == "z")
	{
		backrZ = ret;
		set(0,0,ret, 0,0,0);
	}
	else
	{
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
	}

	printf("%p %ld\n", innerModel, (long int)innerModel);
	if (innerModel != nullptr)
		innerModel->cleanupTables();
	return ret;
}

QVec InnerModelJoint::unitaryAxis()
{
	if( axis == "x") return QVec::vec3(1,0,0);
	if( axis == "y") return QVec::vec3(0,1,0);
	if( axis == "z") return QVec::vec3(0,0,1);
	return QVec::zeros(3);
}

InnerModelNode * InnerModelJoint::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelJoint *ret;
	if (axis == "x")
	{
		ret = new InnerModelJoint(id, backl,0,0, backh,0,0, backtX, backtY, backtZ, backrX, 0, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "y")
	{
		ret = new InnerModelJoint(id, 0,backl,0, 0,backh,0, backtX, backtY, backtZ, 0, backrY, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "z")
	{
		ret = new InnerModelJoint(id, 0,0,backl, 0,0,backh, backtX, backtY, backtZ, 0, 0, backrZ, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else
	{
		fprintf(stderr, "InnerModel internal error: invalid axis %s.\n", axis.c_str());
		exit(-1);
	}

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
	ret->setAngle(getAngle());

	return ret;
}
