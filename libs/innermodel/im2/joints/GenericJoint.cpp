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
// GenericJoint
// ------------------------------------------------------------------------------------------------

GenericJoint::GenericJoint(
	QString id_,
	JointType jtype_,
	float tx_,
	float ty_,
	float tz_,
	float rx_,
	float ry_,
	float rz_,
	uint32_t port_,
	Node *parent_ )
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
}



GenericJoint::~GenericJoint()
{
}



void GenericJoint::print(bool verbose)
{
	printf("GenericJoint: %s\n", qPrintable(id));
	if (verbose)
	{
		im_pose.print(qPrintable(id));
		im_pose.getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void GenericJoint::save(QTextStream &out, int tabs)
{
	QSet<Node*>::iterator c;
	
	if (id == "root")
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<innerModel>\n";
		for ( c=im_children.begin() ; c!=im_children.end() ; ++c )
			(*c)->save(out, tabs+1);
		for (int i=0; i<tabs; i++) out << "\t";
		out << "</innermodel>\n";
	}
	else
	{
		for (int i=0; i<tabs; i++)
			out << "\t";
		
		out << "<transform id=\"" << id
		    << "\" tx=\"" << QString::number(im_tx, 'g', 10)
		    << "\" ty=\"" << QString::number(im_ty, 'g', 10)
		    << "\" tz=\"" << QString::number(im_tz, 'g', 10)
			<< "\" rx=\"" << QString::number(im_rx, 'g', 10)
			<< "\" ry=\"" << QString::number(im_ry, 'g', 10)
			<< "\" rz=\"" << QString::number(im_rz, 'g', 10)
			<< "\">\n";
		
		for (c=im_children.begin(); c!=im_children.end(); c++)
			(*c)->save(out, tabs+1);

		for (int i=0; i<tabs; i++)
			out << "\t";
		
		out << "</transform>\n";
	}
}



void GenericJoint::setRotation( const float rx_, const float ry_, const float rz_ )
{
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
// 	im_fixed = true;
}



void GenericJoint::setTranslation( const float tx_, const float ty_, const float tz_ )
{
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
// 	im_fixed = true;
}



void GenericJoint::setTransform( const float tx_, const float ty_, const float tz_, const float rx_, const float ry_, const float rz_ )
{
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
// 	im_fixed = true;
}



void GenericJoint::initOSG( osg::Group* graphicsScene )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void GenericJoint::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void GenericJoint::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void GenericJoint::postUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
