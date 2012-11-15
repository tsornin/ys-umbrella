#include "PyramidTestState.h"
#include <vector>

/*
================================
PyramidTestState::Instance

Singleton pattern.
================================
*/
PyramidTestState* PyramidTestState::Instance()
{
	static PyramidTestState the_PyramidTestState;
	return &the_PyramidTestState;
}

/*
================================
PyramidTestState::init
================================
*/
void PyramidTestState::init( Engine* game )
{
	EntityState::init( game );

	const int x = 5;
	const Scalar scale = 50.0;
	const Scalar step = scale * 1.5;
	// Vec2 off = Vec2( -x, x/2 ) * step * 0.5;

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
	for ( int i = 0; i < x-j; ++i ) {
		Vec2 off = Vec2( j-x+1, 0 ) * step * 0.5;

		Rigid* rg = PhysicsState::createRigid( o_rg );
		rg->position = Vec2( i, j ) * step + off;
		rg->gravity = g;
		rg->mask = 0x1;
	}}
}
