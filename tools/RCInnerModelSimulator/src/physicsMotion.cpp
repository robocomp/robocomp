#include "physicsMotion.h"

#include <osg/Shape>



MotionOSG::MotionOSG ( const btTransform& initial, osg::ref_ptr< osg::MatrixTransform >& n )
{
	node = n;
	transform = initial;
}


MotionOSG::~MotionOSG()
{
	node.release();
}


void MotionOSG::getWorldTransform ( btTransform& worldTrans ) const
{
	worldTrans = transform;
}


void MotionOSG::setWorldTransform ( const btTransform& worldTrans )
{
	transform = worldTrans;
	
	btScalar buffer[4][4];
	transform.getOpenGLMatrix( &buffer[0][0] );
	// x = x, y = z, z = -y
	osg::Matrix m(
		+buffer[0][0], +buffer[0][2], -buffer[0][1], +buffer[0][3],
		+buffer[2][0], +buffer[2][2], -buffer[2][1], +buffer[2][3],
		-buffer[1][0], -buffer[1][2], -buffer[1][1], -buffer[1][3],
		+buffer[3][0], +buffer[3][2], -buffer[3][1], +buffer[3][3]
	);
	node->setMatrix( m );
}
