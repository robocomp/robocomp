#include "physicsWorld.h"
#include "physicsMotion.h"



World::World()
{
	// Build the broadphase
	broadphase = new btDbvtBroadphase();

	// Set up the collision configuration and dispatcher
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher ( collisionConfiguration );

	// The actual physics solver
	solver = new btSequentialImpulseConstraintSolver;

	// The world.
	dynamicsWorld = new btDiscreteDynamicsWorld ( dispatcher, broadphase, solver, collisionConfiguration );
	dynamicsWorld->setGravity ( btVector3 ( 0,0,-0.980665f ) );
}


World::~World()
{
	for( uint i = 0 ; i < bodies.size() ; ++i ) {
		dynamicsWorld->removeRigidBody( bodies[i] );
		delete bodies[i]->getMotionState();
		delete bodies[i];
	}
	
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
}


void World::addStaticBody ( btCollisionShape* shape, const btTransform& transform, const btScalar restitution )
{
	btDefaultMotionState* motion = new btDefaultMotionState( transform );
	btRigidBody::btRigidBodyConstructionInfo info( 0.0f, motion, shape, btVector3(0,0,0) );
	btRigidBody* body = new btRigidBody( info );
	body->setRestitution( restitution );
	dynamicsWorld->addRigidBody( body );
	bodies.push_back( body ); 
}


void World::addRigidBody( btCollisionShape* shape, btMotionState* motion, const btScalar restitution, const btScalar mass )
{
	btVector3 inertia(0,0,0);
	shape->calculateLocalInertia( mass, inertia );
	btRigidBody::btRigidBodyConstructionInfo info( mass, motion, shape, inertia );
	btRigidBody* rigid = new btRigidBody( info );
	rigid->setRestitution( restitution );
	dynamicsWorld->addRigidBody( rigid );
	bodies.push_back( rigid );
}


void World::update ( const float delta )
{
	dynamicsWorld->stepSimulation ( delta, 20 );

// 	btTransform trans;
// 	printf( "------------------------------------------------------------\n" );
// 	for ( uint i = 0 ; i < bodies.size() ; ++i ) {
// 		bodies[i]->getMotionState()->getWorldTransform ( trans );
// 		printf ( "Position of object #%d: (%f, %f, %f)\n", i, trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ() );
// 	}
}
