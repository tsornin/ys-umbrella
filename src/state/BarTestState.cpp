#include "BarTestState.h"
#include <vector>

/*
================================
BarTestState::Instance

Singleton pattern.
================================
*/
BarTestState* BarTestState::Instance()
{
	static BarTestState the_BarTestState;
	return &the_BarTestState;
}

/*
================================
BarTestState::init
================================
*/
void BarTestState::init( Engine* game )
{
	EntityState::init( game );

	const int x = 8 + 2;
	const Scalar step = 60.0;
	Vec2 off = Vec2( -x, -x ) * step * 0.5;

	const Vec2 g( 0, -0.6 );

	Scalar mass = 1024.0;
	Scalar k = 0.75;

	// Verlet masses
	std::vector < Verlet* > vls;
	for ( int j = 0; j < x; ++j ) {
		Verlet* vl = PhysicsState::createVerlet( 1 );
		vl->putPosition( Vec2( j, j ) * step + off );
		vl->setMass( mass );
		vl->setGravity( g );
		vls.push_back( vl );
		mass *= k;
	}

	// Distance constraints
	std::vector < Distance* > dcs;
	for ( int j = 0; j < x-1; ++j ) {
		Distance* dc = PhysicsState::createDistance( vls[j], vls[j+1], 1 );
		dc->setType( DC_HARD );
		dc->setPower( 1.0 );
		dcs.push_back( dc );
	}

	// Angular constraints
	for ( int j = 0; j < x-2; ++j ) {
		Angular* ac = PhysicsState::createAngular( dcs[j], dcs[j+1] );
	}

	// Pin bottom
	vls[0]->setLinearEnable( false );
	vls[1]->setLinearEnable( false );
}
