#include "FrictionTestState.h"
#include <vector>

/*
================================
FrictionTestState::Instance

Singleton pattern.
================================
*/
FrictionTestState* FrictionTestState::Instance()
{
	static FrictionTestState the_FrictionTestState;
	return &the_FrictionTestState;
}

/*
================================
FrictionTestState::init
================================
*/
void FrictionTestState::init( Engine* game )
{
	EntityState::init( game );

	const int x = 2;
	// const int x = 1;
	const Scalar scale = 50.0;
	const Scalar step = scale * 1.5;
	Vec2 off = Vec2( 0, 0.5 ) * step;

	const Vec2 g( 0, -0.3 );

	MeshOBJ o_frame;
	o_frame.load( Path( "level/pong/", "frame.obj" ) );
	o_frame.setScale( 50 );

	Rigid* frame = PhysicsState::createRigid( o_frame, 1 );
	frame->setPosition( Vec2( 0, -600 ) );
	frame->setLinearEnable( false );
	frame->setAngularEnable( false );
	// frame->setFriction( 0.5 );

	// frame = PhysicsState::createRigid( o_frame, 1 );
	// frame->setPosition( Vec2( 0, -600 ) );
	// frame->setAngle( 1.57 );
	// frame->setLinearEnable( false );
	// frame->setAngularEnable( false );
	// // frame->setFriction( 0.5 );

	MeshOBJ o_rg;
	o_rg.load( Path( "level/test/", "4gon.obj" ) );
	// o_rg.load( Path( "level/test/", "3gon.obj" ) );
	o_rg.setScale( 50 );

	for ( int j = 0; j < x; ++j ) {
		Rigid* rg = PhysicsState::createRigid( o_rg, 1 );
		rg->setPosition( Vec2( 0, j ) * step + off );
		rg->setGravity( g );
		rg->setBounce( 0.5 );
		rg->setFriction( 0.5 );
		// rg->setAngularVelocity( 5 );
	}
}
