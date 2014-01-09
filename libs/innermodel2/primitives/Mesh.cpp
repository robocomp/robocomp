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
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osgDB/ReadFile>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Mesh
// ------------------------------------------------------------------------------------------------

Mesh::Mesh(
	QString id_,
	QString path_,
	float tx_, float ty_, float tz_,
	float rx_, float ry_, float rz_,
	float sx_, float sy_, float sz_,
	Node* parent_)
: Primitive( id_, parent_ )
{
	path = path_;
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
	sx = sx_;
	sy = sy_;
	sz = sz_;
	osg_transform = NULL;
}



Mesh::~Mesh()
{
}



void Mesh::print(bool verbose)
{
}



void Mesh::save(QTextStream &out, int tabs)
{
}



void Mesh::initOSG( osg::Group* graphicsScene )
{
	// Compute the local transformation matrix
	osg_transform = new osg::MatrixTransform;
	RTMat matRT = RTMat();
	matRT.setR ( rx, ry, rz );
	matRT.setTr( tx, ty, tz );
	osg::Matrix matS = osg::Matrix::scale( sx, sy, sz );
	osg_transform->setMatrix( matS * QMatToOSGMat4(matRT) );
	
	// Load the mesh
// 	osgDB::Options options;
// 	options.setObjectCacheHint( osgDB::Options::CACHE_ALL );
	const std::string fullPath = (xml_base_path + "/" + path).toStdString();
	printf( "Loading mesh %s\n", fullPath.c_str() );
	osg::Node* osgMesh = osgDB::readNodeFile( fullPath/*, &options*/ );
	if( osgMesh == NULL ) {
		printf( "Could load mesh file %s\n", fullPath.c_str() );
		throw Exception( "Could load mesh file " + fullPath );
	}
	
	// Add the mesh to the tree
	osg_transform->addChild( osgMesh );
	
	Body* b = dynamic_cast<Body*>( im_parent );
	b->osg_transform->addChild( osg_transform );
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void Mesh::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void Mesh::preUpdate( const float elapsed )
{
}



void Mesh::postUpdate( const float elapsed )
{
}

}
