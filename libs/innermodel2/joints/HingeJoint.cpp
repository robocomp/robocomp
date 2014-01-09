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
// Joint
// ------------------------------------------------------------------------------------------------

HingeJoint::HingeJoint(
	QString id_,
	JointType jtype_,
	float tx_,
	float ty_,
	float tz_,
	float rx_,
	float ry_,
	float rz_,
	float min_,
	float max_,
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
	im_axis = axis_;
}



HingeJoint::~HingeJoint()
{
}



void HingeJoint::print(bool verbose)
{
	printf("Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		im_pose.print(qPrintable(id));
		im_pose.getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void HingeJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}



float HingeJoint::getAngle() const
{
	return im_angle;
}



float HingeJoint::setAngle(float angle)
{
	if (angle >= im_max) {
		im_angle = im_max;
	}
	else if (angle <= im_min) {
		im_angle = im_min;
	}
	else {
		im_angle = angle;
	}
	
	RTMat m1, m2;
	m1.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
// 	if (im_axis == "x") {
// 		im_pose.set( im_angle+im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
// 	}
// 	else if (im_axis == "y") {
// 		im_pose.set( im_rx, im_angle+im_ry, im_rz, im_tx, im_ty, im_tz );
// 	}
// 	else if (im_axis == "z") {
// 		im_pose.set( im_rx, im_ry, im_angle+im_rz, im_tx, im_ty, im_tz );
// 	}
// 	else {
// 		qFatal("internal error, no such axis %s\n", im_axis.c_str());
// 	}
	if (im_axis == "x") {
		m2.set( +im_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	}
	else if (im_axis == "y") {
		m2.set( 0.0f, +im_angle, 0.0f, 0.0f, 0.0f, 0.0f );
	}
	else if (im_axis == "z") {
		m2.set( 0.0f, 0.0f, +im_angle, 0.0f, 0.0f, 0.0f );
	}
	else if (im_axis == "-x") {
		m2.set( -im_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	}
	else if (im_axis == "-y") {
		m2.set( 0.0f, -im_angle, 0.0f, 0.0f, 0.0f, 0.0f );
	}
	else if (im_axis == "-z") {
		m2.set( 0.0f, 0.0f, -im_angle, 0.0f, 0.0f, 0.0f );
	}
	else {
		qFatal("internal error, no such axis %s\n", im_axis.c_str());
	}
	im_pose = m1 * m2;
	return im_angle;
}



QVec HingeJoint::unitaryAxis()
{
	if( im_axis == "x") return QVec::vec3(+1,0,0);
	else if( im_axis == "y") return QVec::vec3(0,+1,0);
	else if( im_axis == "z") return QVec::vec3(0,0,+1);
	else if( im_axis == "-x") return QVec::vec3(-1,0,0);
	else if( im_axis == "-y") return QVec::vec3(0,-1,0);
	else if( im_axis == "-z") return QVec::vec3(0,0,-1);
	else return QVec::zeros(3);
}



void HingeJoint::initOSG( osg::Group* graphicsScene )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initOSG( graphicsScene );
	}
}



void HingeJoint::initBullet( btDynamicsWorld* physicsWorld )
{
	// Initialize the children
	foreach( IM2::Node* child, im_children ) {
		child->initBullet( physicsWorld );
	}
	
	if( (im_jtype == Free) || (im_jtype == Motor) ) {
		// Reference frame of the parent
		Body* nodeA = dynamic_cast<Body*>( im_parent );
		btRigidBody* bodyA = nodeA->bt_body;
		const btVector3 pivotA( im_tx, im_ty, -im_tz );
		
		// Reference frame of the child
		Body* nodeB = dynamic_cast<Body*>( *(im_children.begin()) );
		btRigidBody* bodyB = nodeB->bt_body;
		const QVec poseB = nodeB->im_pose.getTr();
		const btVector3 pivotB( -poseB.x(), -poseB.y(), +poseB.z() );
		
		// Axis of rotation
		btVector3 rot_axis;
	// 	if( im_axis == "x") rot_axis = btVector3(+1,0,0);
	// 	else if( im_axis == "y") rot_axis = btVector3(0,+1,0);
	// 	else if( im_axis == "z") rot_axis = btVector3(0,0,-1);
	// 	else if( im_axis == "-x") rot_axis = btVector3(-1,0,0);
	// 	else if( im_axis == "-y") rot_axis = btVector3(0,-1,0);
	// 	else if( im_axis == "-z") rot_axis = btVector3(0,0,+1);
		if( im_axis == "x") rot_axis = btVector3(1,0,0);
		else if( im_axis == "y") rot_axis = btVector3(0,1,0);
		else if( im_axis == "z") rot_axis = btVector3(0,0,1);
		else if( im_axis == "-x") rot_axis = btVector3(1,0,0);
		else if( im_axis == "-y") rot_axis = btVector3(0,1,0);
		else if( im_axis == "-z") rot_axis = btVector3(0,0,1);
		
		// Build the constraint
		btHingeConstraint* joint = new btHingeConstraint(
			*bodyA,
			*bodyB,
			pivotA,
			pivotB,
			rot_axis,
			rot_axis );
		const bool DISABLE_COLLISIONS = false;
		const float SOFTNESS = 0.9f;		// Unused?
		const float BIAS = 0.3f;		// Error multiplier, 0 is softer
		const float RELAXATION = 1.0f;		// Bounciness: 0 no bounce, 1 full bounce
		joint->setLimit( im_min, im_max, SOFTNESS, BIAS, RELAXATION );
		
	// 	// Configure the motor
	// 	if( im_port > 0 ) {
	// 		printf( "Adding a motor\n" );
	// // 		joint->setAngularOnly( true );
	// 		joint->enableMotor( true );
	// 		joint->enableAngularMotor( true, 0.0f, 100000.0f );
	// 	}
		
		// Add the constraint to the world
		physicsWorld->addConstraint( joint, DISABLE_COLLISIONS );
		bt_constraint = joint;
		
		bodyA->addConstraintRef( joint );
		bodyB->addConstraintRef( joint );
		
// 		const btTransform& frameA = joint->getAFrame();
// 		const btTransform& frameB = joint->getBFrame();
// 		printf( "Joint %s\n", id.toLatin1().data() );
// 		printf( "\tPivot A: %f, %f, %f\n", pivotA.x(), pivotA.y(), pivotA.z() );
// 		printf( "\tAxis A: %f, %f, %f\n", rot_axis.x(), rot_axis.y(), rot_axis.z() );
// 		printf( "\t%f %f %f %f\n", frameA.getBasis()[0][0], frameA.getBasis()[0][1], frameA.getBasis()[0][2], frameA.getOrigin().x() );
// 		printf( "\t%f %f %f %f\n", frameA.getBasis()[1][0], frameA.getBasis()[1][1], frameA.getBasis()[1][2], frameA.getOrigin().x() );
// 		printf( "\t%f %f %f %f\n", frameA.getBasis()[2][0], frameA.getBasis()[2][1], frameA.getBasis()[2][2], frameA.getOrigin().x() );
// 		printf( "\n" );
// 		printf( "\tPivot B: %f, %f, %f\n", pivotB.x(), pivotB.y(), pivotB.z() );
// 		printf( "\tAxis B: %f, %f, %f\n", rot_axis.x(), rot_axis.y(), rot_axis.z() );
// 		printf( "\t%f %f %f %f\n", frameB.getBasis()[0][0], frameB.getBasis()[0][1], frameB.getBasis()[0][2], frameB.getOrigin().x() );
// 		printf( "\t%f %f %f %f\n", frameB.getBasis()[1][0], frameB.getBasis()[1][1], frameB.getBasis()[1][2], frameB.getOrigin().x() );
// 		printf( "\t%f %f %f %f\n", frameB.getBasis()[2][0], frameB.getBasis()[2][1], frameB.getBasis()[2][2], frameB.getOrigin().x() );
	}
}



void HingeJoint::preUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->preUpdate( elapsed );
	}
	
// 	// Configure the motor
// 	if( im_port > 0 ) {
// 		btHingeConstraint* joint = dynamic_cast<btHingeConstraint*>( bt_constraint );
// 		joint->enableMotor( true );
// 		joint->enableAngularMotor( true, 0.0f, 1000.0f );
// 	}
}



void HingeJoint::postUpdate( const float elapsed )
{
	// Update the children
	QSet<Node*>::iterator it;
	for ( it = im_children.begin() ; it != im_children.end(); ++it ) {
		(*it)->postUpdate( elapsed );
	}
}

}
