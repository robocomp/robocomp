#pragma once


#include <bullet/btBulletDynamicsCommon.h>
#include <osg/MatrixTransform>


class MotionOSG : public btMotionState
{
private:
	osg::ref_ptr<osg::MatrixTransform> node;
	btTransform transform;
	
public:
	MotionOSG( const btTransform& initial, osg::ref_ptr<osg::MatrixTransform>& n );
	virtual ~MotionOSG();
	
	virtual void getWorldTransform ( btTransform& worldTrans ) const;
	virtual void setWorldTransform ( const btTransform& worldTrans );
};
