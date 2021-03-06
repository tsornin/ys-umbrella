#include "StringTestState.h"
#include <vector>

/*
================================
StringTestState::Instance

Singleton pattern.
================================
*/
StringTestState* StringTestState::Instance()
{
	static StringTestState the_StringTestState;
	return &the_StringTestState;
}

/*
================================
StringTestState::init
================================
*/
void StringTestState::init( Engine* game )
{
	EntityState::init( game );

	const int x = 40;
	const Scalar step = 20.0;
	Vec2 off = Vec2( -x, x/2 ) * step * 0.5;

	const Vec2 g( 0, -0.6 );

	const Scalar mass = 512.0;

	// Verlet masses
	std::vector < Verlet* > vls;
	for ( int i = 0; i < x; ++i ) {
		Verlet* vl = PhysicsState::createVerlet();
		vl->putPosition( Vec2( i, 0 ) * step + off );
		vl->mass = mass;
		vl->gravity = g;
		vls.push_back( vl );
	}

	// Distance constraints
	for ( int i = 0; i < x-1; ++i ) {
		Distance* dc = PhysicsState::createDistance( vls[i], vls[i+1] );
		dc->type = DC_PULL;
		dc->power = 1.0;
	}

	// Pin ends
	vls[0]->linear_enable = false;
	vls[x-1]->linear_enable = false;

	// Pin center (we should see two islands)
	vls[x/2]->linear_enable = false;
}
