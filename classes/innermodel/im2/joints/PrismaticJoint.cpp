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

#include "../innermodel.h"

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// PrismaticJoint
// ------------------------------------------------------------------------------------------------

PrismaticJoint::PrismaticJoint(
	QString id_,
	JointType jtype_,
	float tx_, float ty_, float tz_,
	float rx_, float ry_, float rz_,
	float min_,
	float max_,
	float val_,
	float offset_,
 	std::string axis_,
	uint32_t port_,
	Body *parent_)
: Joint( id_, jtype_, port_, parent_ )
{
	// Joint pose
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	im_pose.set( rx_, ry_, rz_, tx_, ty_, tz_ );
	
	// Specific joint parameters
	im_min = min_;
	im_max = max_;
	im_offset = offset_;
	im_axis = axis_;
	setPosition(val_);
}



PrismaticJoint::~PrismaticJoint()
{
}



void PrismaticJoint::print(bool verbose)
{
	printf("Prismatic Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		im_pose.print(qPrintable(id));
		im_pose.getTr().print(id+"_T");
	}
}



void PrismaticJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}



float PrismaticJoint::getPosition() const
{
	return im_value;
}



float PrismaticJoint::setPosition( const float value )
{
	if (value >= im_max) {
		im_value = im_max;
	}
	else if (value <= im_min) {
		im_value = im_min;
	}
	else {
		im_value = value;
	}
	
	if (im_axis == "x") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx+im_value+im_offset, im_ty, im_tz );
	}
	else if (im_axis == "y") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty+im_value+im_offset, im_tz );
	}
	else if (im_axis == "z") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz+im_value+im_offset );
	}
	else if (im_axis == "-x") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx-im_value-im_offset, im_ty, im_tz );
	}
	else if (im_axis == "-y") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty-im_value-im_offset, im_tz );
	}
	else if (im_axis == "-z") {
		im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz-im_value-im_offset );
	}
	else {
		qFatal("internal error, no such axis %s\n", im_axis.c_str());
	}
	return im_value;
}



void PrismaticJoint::initOSG( osg::Group* graphicsScene )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void PrismaticJoint::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void PrismaticJoint::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void PrismaticJoint::postUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
