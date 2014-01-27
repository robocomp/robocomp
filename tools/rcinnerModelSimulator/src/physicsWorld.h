#pragma once


#include <vector>

#include <bullet/btBulletDynamicsCommon.h>
#include <osg/MatrixTransform>



class World
{
private:
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	
	std::vector<btRigidBody*> bodies;
	
public:
	World();
	~World();
	
	void addStaticBody( btCollisionShape* shape, const btTransform& transform, const btScalar restitution );
	void addRigidBody( btCollisionShape* shape, btMotionState* motion, const btScalar restitution, const btScalar mass );
	//void addFromMesh(  );
	
	void update( const float delta );
};
