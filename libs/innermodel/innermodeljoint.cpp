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

#include "innermodel.h"
#include "innermodeljoint.h"

class InnerModel;

InnerModelJoint::InnerModelJoint() : InnerModelTransform("invalid",QString("static"), 0,0,0, 0,0,0, 0, NULL)
{
	throw std::string("Can't actually build InnerModelJoint using the default constructor");
}

InnerModelJoint::InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
	QMutexLocker l(mutex);
	#if FCL_SUPPORT==1
	collisionObject = NULL;
	#endif

	// 		set(rx_, ry_, rz_, tx_, ty_, tz_);
	backlX = lx_;
	backlY = ly_;
	backlZ = lz_;
	backhX = hx_;
	backhY = hy_;
	backhZ = hz_;
	min = min_;
	max = max_;
	home = home_;
	hx = hy = hz =lx = ly = lz = NULL;
	port = port_;
	axis = axis_;
	if (axis == "x")
	{
		update(min, 0, 0, max, 0, 0);
	}
	else if (axis == "y")
	{
		update(0, min, 0, 0, max, 0);
	}
	else if (axis == "z")
	{
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
	QMutexLocker l(mutex);
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
	QMutexLocker l(mutex);
	QList<InnerModelNode*>::iterator c;
	//<joint id="head_yaw_joint" port="10067" axis="z" home="0" min="-1" max="1">
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<joint id=\"" << id << "\" port=\"" << port << "\" axis=\"" <<QString::fromStdString( axis)<<"\" home=\""<< QString::number(home, 'g', 10)
	<<"\" min=\""<< QString::number(min, 'g', 10)<<"\" max=\""<< QString::number(max, 'g', 10)
	<< "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10)
	<<"\"  rx=\""<< QString::number(backrX, 'g', 10) <<"\" ry=\""<< QString::number(backrY, 'g', 10) <<"\" rz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
	for (c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);

	for (int i=0; i<tabs; i++) out << "\t";
	out << "</joint>\n";
}

void InnerModelJoint::setUpdatePointers(float *lx_, float *ly_, float *lz_, float *hx_, float *hy_, float *hz_)
{
	QMutexLocker l(mutex);
	lx = lx_;
	ly = ly_;
	lz = lz_;
	hx = hx_;
	hy = hy_;
	hz = hz_;
	fixed = false;
}

void InnerModelJoint::update()
{
	QMutexLocker l(mutex);
	if (!fixed)
	{
		if (lx) backtX = *tx;
		if (ly) backtY = *ty;
		if (lz) backtZ = *tz;
		if (rx) backhX = *hx;
		if (ry) backhY = *hy;
		if (rz) backhZ = *hz;
	}
	updateChildren();
}

void InnerModelJoint::update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_)
{
	QMutexLocker l(mutex);
	backhX = hx_; backhY = hy_; backhZ = hz_;
	backlX = lx_; backlY = ly_; backlZ = lz_;
	fixed = true;
}

float InnerModelJoint::getAngle()
{
	printf("getAngle from %p\n", this);
	QMutexLocker l(mutex);
	return backrZ;
}

float InnerModelJoint::setAngle(float angle, bool force)
{
	printf("setAngle from %p\n", this);
	QMutexLocker l(mutex);
	float ret;
	if ((angle <= max and angle >= min) or force)
	{
		ret = angle;
	}
	else if (angle > max)
	{
		ret = max;
	}
	else
	{
		ret = min;
	}

	backrZ = ret;

	if (axis == "x")
	{
		printf("x\n");
		print("m");
		set(ret,0,0, 0,0,0);
	}
	else if (axis == "y")
	{
		printf("y\n");
		set(0,ret,0, 0,0,0);
	}
	else if (axis == "z")
	{
		printf("z\n");
		set(0,0,ret, 0,0,0);
	}
	else
	{
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
	}

	printf("%p %ld\n", innerModel, (long int)innerModel);
	if (innerModel)
		innerModel->cleanupTables();
	return ret;
}

QVec InnerModelJoint::unitaryAxis()
{
	QMutexLocker l(mutex);
	if( axis == "x") return QVec::vec3(1,0,0);
	if( axis == "y") return QVec::vec3(0,1,0);
	if( axis == "z") return QVec::vec3(0,0,1);
	return QVec::zeros(3);
}

InnerModelNode * InnerModelJoint::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	QMutexLocker l(mutex);
	InnerModelJoint *ret;
	if (axis == "x")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, backrZ, 0, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "y")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, 0, backrZ, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "z")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, 0, 0, backrZ, min, max, port, axis, home, (InnerModelTransform *)parent);
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

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}
	return ret;
}
