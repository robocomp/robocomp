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
#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>
#include <osg/MatrixTransform>
#include <osgbCollision/CollisionShapes.h>

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// Body
// ------------------------------------------------------------------------------------------------

Body::Body( QString id_, const RTMat& pose_, const BodyParams& params_, Node* parent_ )
: Node( id_, parent_ )
{
	this->bt_params = params_;
	this->im_pose = pose_;
	
	bt_motion = NULL;
	bt_body = NULL;
	bt_shape = NULL;
}



Body::~Body() {
	delete bt_shape;
	delete bt_body;
}



void Body::print( bool verbose )
{
	printf( "Body: %s\n", qPrintable(id) );
	if (verbose)
	{
		im_pose.print(qPrintable(id));
		im_pose.getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void Body::save( QTextStream &out, int tabs )
{
	QList<Node*>::iterator c;
	
	for (int i=0; i<tabs; i++) out << "\t";
	
// 	body->getMotionState()->getWorldTransform( transform );
// 	const btVector3 translation = transform.getOrigin();
// 	const btQuaternion rotation = transform.getRotation();
	
// 	out << "<body id=\"" << id
// 		<< "\" tx=\"" << QString::number( translation.x(), 'g', 10)
// 		<< "\" ty=\"" << QString::number( translation.y(), 'g', 10)
// 		<< "\" tz=\"" << QString::number( translation.z(), 'g', 10)
// 		<< "\" rx=\"" << QString::number( rotation.x(), 'g', 10)
// 		<< "\" ry=\"" << QString::number( rotation.y(), 'g', 10)
// 		<< "\" rz=\"" << QString::number( rotation.z(), 'g', 10)
// 		<< "\">\n";
	
	for (int i=0; i<tabs; i++)
		out << "\t";
	out << "</body>\n";
}



void Body::initOSG( osg::Group* graphicsScene )
{
	printf( "INIT OSG\n" );
	
	// Initialize the body attributes
	osg_transform = new osg::MatrixTransform;
	graphicsScene->addChild( osg_transform );
	
	// Configure the rendering mode
	osg_polygonMode = new osg::PolygonMode;
	osg_polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
	osg_transform->getOrCreateStateSet()->setAttributeAndModes( osg_polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
	osg_transform->getOrCreateStateSet()->setMode( GL_RESCALE_NORMAL, osg::StateAttribute::ON );
	
// 	// Add the body components
// 	QHash<QString, MeshParams>::iterator it;
// 	for( it = im_meshes.begin() ; it != im_meshes.end() ; ++it ) {
// 		// Compute the local transformation matrix
// 		osg::MatrixTransform* mt = new osg::MatrixTransform;
// 		RTMat matRT = RTMat();
// 		matRT.setR ( it.value().rx, it.value().ry, it.value().rz );
// 		matRT.setTr( it.value().tx, it.value().ty, it.value().tz );
// 		osg::Matrix matS = osg::Matrix::scale( it.value().sx, it.value().sy, it.value().sz );
// 		mt->setMatrix( matS * QMatToOSGMat4(matRT) );
// 		
// 		// Load the mesh
// // 		osgDB::Options options;
// // 		options.setObjectCacheHint( osgDB::Options::CACHE_ALL );
// 		osg::Node* osgMesh = osgDB::readNodeFile( it.value().path.toStdString()/*, &options*/ );
// 		osg_meshes[it.key()] = osgMesh;
// 		
// 		// Add the mesh to the tree
// 		mt->addChild( osgMesh );
// 		osg_transform->addChild( mt );
// 	}
	
// 	// Render the geometrical center of the body
// 	osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0),0.01 );
// 	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable( unitSphere );
// 	unitSphereDrawable->setColor( osg::Vec4(0.0f, 1.0f, 0.0f, 0.5f) );
// 	osg::Geode* unitSphereGeode = new osg::Geode();
// 	unitSphereGeode->addDrawable(unitSphereDrawable); 
// 	osg_transform->addChild( unitSphereGeode );
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void Body::initBullet( btDynamicsWorld* physicsWorld )
{
	// Common physics parameters
	QMatToBullet( im_absolute, bt_transform );
	bt_motion = new btDefaultMotionState( bt_transform );
	btVector3 inertia(0,0,0);
	
	// Use GImpact for the collision detection
	// Slower, but much more precise
	btTriangleMeshShape* tri = osgbCollision::btTriMeshCollisionShapeFromOSG( osg_transform );
	btStridingMeshInterface* str = tri->getMeshInterface();
	btGImpactMeshShape* gim = new btGImpactMeshShape( str );
	//gim->setLocalScaling( btVector3( 0.98f, 0.98f, 0.98f ) );
	gim->updateBound();
	gim->postUpdate();
	gim->setMargin( 1e-2 );
	bt_shape = gim;
	
	// Create the rigid body
	if( bt_params.type == Static ) {
// 		bt_shape = osgbCollision::btTriMeshCollisionShapeFromOSG( osg_transform );
		btRigidBody::btRigidBodyConstructionInfo info( 0.0f, bt_motion, bt_shape, inertia );
		bt_body = new btRigidBody( info );
	}
	else if ( bt_params.type == Dynamic ) {
		//bt_shape = osgbCollision::btBoxCollisionShapeFromOSG( osg_transform );
		bt_shape->calculateLocalInertia( bt_params.mass, inertia );
		btRigidBody::btRigidBodyConstructionInfo info( bt_params.mass, bt_motion, bt_shape, inertia );
		bt_body = new btRigidBody( info );
	}
	else if( bt_params.type == Kinematic ) {
		// TODO: finish this
// 		bt_shape = osgbCollision::btBoxCollisionShapeFromOSG( osg_transform );
		btRigidBody::btRigidBodyConstructionInfo info( 0.0f, bt_motion, bt_shape, inertia );
		bt_body = new btRigidBody( info );
	}
	else if( bt_params.type == Ghost ) {
		// Do nothing, ghost bodies are only for rendering
	}
	else {
		throw Exception( "Unknown body type!" );
	}
	
	// Configure the body parameters
	bt_body->setRestitution( bt_params.restitution );
	bt_body->setFriction( bt_params.friction );
	bt_body->setRollingFriction( bt_params.friction );
	bt_body->setActivationState( DISABLE_DEACTIVATION );
	
	// Add the body to the world
	physicsWorld->addRigidBody( bt_body );
	
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
}



void Body::preUpdate( const float elapsed )
{
	// If the object has been moved, update the transform
	if( bt_params.type != Dynamic ) {
// 		printf( "Body %s\n", qPrintable( id ) );
// 		printf( "\t%f %f %f %f\n", im_absolute(0, 0), im_absolute(0, 1), im_absolute(0, 2), im_absolute(0, 3) );
// 		printf( "\t%f %f %f %f\n", im_absolute(1, 0), im_absolute(1, 1), im_absolute(1, 2), im_absolute(1, 3) );
// 		printf( "\t%f %f %f %f\n", im_absolute(2, 0), im_absolute(2, 1), im_absolute(2, 2), im_absolute(2, 3) );
		QMatToBullet( im_absolute, bt_transform );
		bt_motion->setWorldTransform( bt_transform );
	}
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void Body::postUpdate( const float elapsed )
{
	// Update the body transform
	bt_motion->getWorldTransform( bt_transform );
// 	BulletToQMat( bt_transform, im_absolute );
// 	osg_transform->setMatrix( QMatToOSGMat4( im_absolute ) );
	osg_transform->setMatrix( osgbCollision::asOsgMatrix( bt_transform ) );
	
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
