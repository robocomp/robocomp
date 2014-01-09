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

// OSG includes
#include <osg/MatrixTransform>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Laser
// ------------------------------------------------------------------------------------------------

Laser::Laser(
	QString id_,
	uint32_t port_,
	uint32_t min_,
	uint32_t max_,
	float angle_,
	uint32_t measures_,
	QString ifconfig_,
	Node *parent_)
: Sensor(id_, port_, parent_)
{
	im_min = min_;
	im_max = max_;
	im_measures = measures_;
	im_angle = angle_;
	im_ifconfig = ifconfig_;
}



Laser::~Laser()
{
}



void Laser::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<laser id=\"" << id << "\" />\n";
}



void Laser::print(bool verbose)
{
	if (verbose) printf("LASER.");
}



void Laser::initOSG( osg::Group* graphicsScene )
{
	osg_transform = new osg::MatrixTransform;
	graphicsScene->addChild( osg_transform );
	
	osg_node = new osg::Switch();
	osg_transform->addChild( osg_node );
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void Laser::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void Laser::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void Laser::postUpdate( const float elapsed )
{
	osg_transform->setMatrix( QMatToOSGMat4(im_pose) );
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
