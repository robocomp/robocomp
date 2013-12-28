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
// BallJoint
// ------------------------------------------------------------------------------------------------

BallJoint::BallJoint(
	QString id_,
	JointType jtype_,
	float lx_,
	float ly_,
	float lz_,
	float hx_,
	float hy_,
	float hz_,
	float tx_,
	float ty_,
	float tz_,
	float rx_,
	float ry_,
	float rz_,
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
	im_lx = lx_;
	im_ly = ly_;
	im_lz = lz_;
	im_hx = hx_;
	im_hy = hy_;
	im_hz = hz_;
}



BallJoint::~BallJoint()
{
}



void BallJoint::print(bool verbose)
{
	printf("Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		im_pose.print(qPrintable(id));
		im_pose.getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void BallJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}



void BallJoint::setLower( float lx_, float ly_, float lz_ )
{
	im_lx = lx_;
	im_ly = ly_;
	im_lz = lz_;
}



void BallJoint::setUpper( const float hx_, const float hy_, const float hz_ )
{
	im_hx = hx_;
	im_hy = hy_;
	im_hz = hz_;
}



void BallJoint::setAngle( const float cx_, const float cy_, const float cz_ )
{
	im_cx = cx_;
	im_cy = cy_;
	im_cz = cz_;
	
	im_pose.set( im_rx+im_cx, im_ry+im_cy, im_rz+im_cz, im_tx, im_ty, im_tz );
}



void BallJoint::getLower( float& lx_, float& ly_, float& lz_ ) const
{
	lx_ = im_lx;
	ly_ = im_ly;
	lz_ = im_lz;
}



void BallJoint::getUpper( float& hx_, float& hy_, float& hz_ ) const
{
	hx_ = im_hx;
	hy_ = im_hy;
	hz_ = im_hz;
}



void BallJoint::getAngle( float& cx_, float& cy_, float& cz_ ) const
{
	cx_ = im_cx;
	cy_ = im_cy;
	cz_ = im_cz;
}



void BallJoint::initOSG( osg::Group* graphicsScene )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void BallJoint::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
	
// 	Body* nodeA = dynamic_cast<Body*>( im_parent );
// 	btRigidBody* bodyA = nodeA->bt_body;
// 	
// 	Body* nodeB = dynamic_cast<Body*>( *(im_children.begin()) );
// 	btRigidBody* bodyB = nodeB->bt_body;
// 	
// 	bt_constraint = new btPoint2PointConstraint(
// 		*bodyA,
// 		*bodyB,
// 		btVector3( im_tx, im_ty, -im_tz ),
// 		btVector3( 0, 0, 0 ) );
// 	const bool DISABLE_COLLISIONS = true;
// 	viewer->getDynamicsWorld()->addConstraint( bt_constraint, DISABLE_COLLISIONS );
// 	
// 	printf( "Ball joint %s: %p\n", id.toLatin1().data(), bt_constraint );

	// Reference frame of the parent
	Body* nodeA = dynamic_cast<Body*>( im_parent );
	btRigidBody* bodyA = nodeA->bt_body;
	const QVec poseA = nodeA->im_pose.getTr();
	const btVector3 pivotA( im_tx, im_ty, -im_tz );
	const btTransform frameA( btQuaternion::getIdentity(), pivotA );

	// Reference frame of the child
	Body* nodeB = dynamic_cast<Body*>( *(im_children.begin()) );
	btRigidBody* bodyB = nodeB->bt_body;
	const QVec poseB = nodeB->im_pose.getTr();
	const btVector3 pivotB( -poseB.x(), -poseB.y(), +poseB.z() );
	const btTransform frameB( btQuaternion::getIdentity(), pivotB );
	
	// Build the constraint
	bt_constraint = new btConeTwistConstraint(
		*bodyA,
		*bodyB,
		frameA,
		frameB );
	const bool DISABLE_COLLISIONS = false;
	const float SOFTNESS = 1.0f;
	const float BIAS = 0.3f;
	const float RELAXATION = 1.0f;
	(dynamic_cast<btConeTwistConstraint*>(bt_constraint))->setLimit( im_hy, im_hz, im_hx, SOFTNESS, BIAS, RELAXATION );
	physicsWorld->addConstraint( bt_constraint, DISABLE_COLLISIONS );
	
	bodyA->addConstraintRef( bt_constraint );
	bodyB->addConstraintRef( bt_constraint );
}



void BallJoint::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
}



void BallJoint::postUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
