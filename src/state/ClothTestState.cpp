#include "ClothTestState.h"
#include <vector>

/*
================================
ClothTestState::Instance

Singleton pattern.
================================
*/
ClothTestState* ClothTestState::Instance()
{
	static ClothTestState the_ClothTestState;
	return &the_ClothTestState;
}

/*
================================
ClothTestState::init
================================
*/
void ClothTestState::init( Engine* game )
{
	EntityState::init( game );

	const int w = 20;
	const int h = 20;
	const Scalar step = 20.0;
	Vec2 off = Vec2( -w, -h ) * step * 0.5;

	const Vec2 g( 0, -0.6 );

	const Scalar mass = 64.0;

	// Verlet masses
	std::vector < std::vector < Verlet* > > vlss;
	for ( int j = 0; j < h; ++j ) {
		vlss.push_back( std::vector < Verlet* >() );
	for ( int i = 0; i < w; ++i ) {
		Verlet* vl = PhysicsState::createVerlet( 1 );
		vl->putPosition( Vec2( i, j ) * step + off );
		vl->setMass( mass );
		vl->setGravity( g );
		vlss[j].push_back( vl );
	}
	}

	// Distance constraints
	for ( int j = 0; j < h; ++j ) {
	for ( int i = 0; i < w; ++i ) {
		Distance* dc;
		if ( i != w-1 ) {
			dc = PhysicsState::createDistance( vlss[j][i], vlss[j][i+1], 1 );
			dc->setType( DC_PULL );
			dc->setPower( 1.0 );
		}
		if ( j != h-1 ) {
			dc = PhysicsState::createDistance( vlss[j][i], vlss[j+1][i], 1 );
			dc->setType( DC_PULL );
			dc->setPower( 1.0 );
		}
	}
	}

	// Pin bottom corners
	vlss[0][0]->setLinearEnable( false );
	vlss[0][w-1]->setLinearEnable( false );

	// Pin top edge
	for ( int i = 0; i < w; ++i ) {
		vlss[h-1][i]->setLinearEnable( false );
	}

	// // Ground
	// MeshOBJ o_gnd;
	// o_gnd.load( Path( "level/pong/", "frame.obj" ) );
	// o_gnd.setScale( 50 );
	// Rigid* gnd = createRigid( o_gnd, ??? );
	// gnd->setLinearEnable( false );
	// gnd->setAngularEnable( false );
}
