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
	frame->position = Vec2( 0, -600 );
	frame->linear_enable = false;
	frame->angular_enable = false;
	frame->mask = 0x1;

	frame = PhysicsState::createRigid( o_frame );
	frame->position = Vec2( 0, -600 );
	frame->angular_position = 1.57;
	frame->linear_enable = false;
	frame->angular_enable = false;
	frame->mask = 0x1;

	MeshOBJ o_rg;
	o_rg.load( Path( "level/test/", "4gon.obj" ) );
	o_rg.setScale( 50 );

	for ( int j = 0; j < x; ++j ) {
		Rigid* rg = PhysicsState::createRigid( o_rg );
		rg->position = Vec2( 0, j ) * step + off;
		rg->gravity = g;
		rg->mask = 0x1;
	}
}
