#include "PhysicsState.h"

/*
================================
PhysicsState::Instance

Singleton pattern.
================================
*/
PhysicsState* PhysicsState::Instance()
{
	static PhysicsState the_PhysicsState;
	return &the_PhysicsState;
}

/*
================================
PhysicsState::init
================================
*/
void PhysicsState::init( Engine* game )
{
	BlankState::init( game );

	next_pid = 0;
	dirty_connected_components = false;
}

/*
================================
PhysicsState::cleanup
================================
*/
void PhysicsState::cleanup()
{
	for ( Rigid* rg : rgs ) delete rg;
	rgs.clear();

	for ( Euler* eu : eus ) delete eu;
	eus.clear();

	for ( Verlet* vl : vls ) delete vl;
	vls.clear();

	for ( Distance* dc : dcs ) delete dc;
	dcs.clear();

	for ( Angular* ac : acs ) delete ac;
	acs.clear();

	connected_components.clear();

	clear_collision_data();

	BlankState::cleanup();
}

/*
================================
PhysicsState::input
================================
*/
void PhysicsState::input( Engine* game )
{
	
}

/*
================================
PhysicsState::update
================================
*/
void PhysicsState::update( Engine* game )
{
	BlankState::update( game );

	expire();

	integrate();

	// Tag Verlet particles with component ID before RG-VL detection
	if ( dirty_connected_components ) {
		find_connected_components();
		dirty_connected_components = false;
	}

	// // During RG-VL detection,
	// // wall constraints are tagged with Verlet component IDs
	// detect_collisions();

	relax_connected_components();
}

/*
================================
PhysicsState::draw
================================
*/
void PhysicsState::draw( Engine* game )
{
	game->rd.drawAxes( 100 );

	// // Display the bounding boxes of the convex list.
	// // (The AABB's of Rigid::shapes are in object space, which isn't useful)
	// for ( auto& pair : vxs ) {
	// 	Convex& vx = pair.second;
	// 	game->rd.drawAABB( vx.getAABB() );
	// }

	for ( Rigid* rg : rgs ) game->rd.drawRigid( *rg );

	for ( Euler* eu : eus ) game->rd.drawEuler( *eu );

	for ( Distance* dc : dcs ) game->rd.drawDistance( *dc );

	// for ( Angular* ac : acs ) game->rd.drawAngular( *ac );

	for ( Verlet* vl : vls ) game->rd.drawVerlet( *vl );
}

/*
================================
PhysicsState::setCaption
================================
*/
void PhysicsState::setCaption( std::ostringstream& buffer )
{
	buffer << " || Physics:";
	buffer << " " << frames_elapsed << " frames elapsed";

	buffer << " " << rgs.size();
	// buffer << "/" << Show number of contacts/islands here.
	buffer << ", " << eus.size();
	buffer << ", " << vls.size() << "-" << dcs.size() << "-" << acs.size();
	buffer << "/" << connected_components.size();
}
