#include "ColumnTestState.h"
#include <vector>

/*
================================
ColumnTestState::Instance

Singleton pattern.
================================
*/
ColumnTestState* ColumnTestState::Instance()
{
	static ColumnTestState the_ColumnTestState;
	return &the_ColumnTestState;
}

/*
================================
ColumnTestState::init
================================
*/
void ColumnTestState::init( Engine* game )
{
	EntityState::init( game );

	const int x = 10;
	const Scalar scale = 50.0;
	const Scalar step = scale * 1.5;
	Vec2 off = Vec2( 0, 0.5 ) * step;

	const Vec2 g( 0, -0.3 );

	MeshOBJ o_frame;
	o_frame.load( Path( "level/pong/", "frame.obj" ) );
	o_frame.setScale( 50 );

	Rigid* frame = PhysicsState::createRigid( o_frame );
	frame->setPosition( Vec2( 0, -600 ) );
	frame->setLinearEnable( false );
	frame->setAngularEnable( false );
	frame->setMask( 1 );

	frame = PhysicsState::createRigid( o_frame );
	frame->setPosition( Vec2( 0, -600 ) );
	frame->setAngle( 1.57 );
	frame->setLinearEnable( false );
	frame->setAngularEnable( false );
	frame->setMask( 1 );

	MeshOBJ o_rg;
	o_rg.load( Path( "level/test/", "4gon.obj" ) );
	o_rg.setScale( 50 );

	for ( int j = 0; j < x; ++j ) {
		Rigid* rg = PhysicsState::createRigid( o_rg );
		rg->setPosition( Vec2( 0, j ) * step + off );
		rg->setGravity( g );
		rg->setMask( 1 );
	}
}
