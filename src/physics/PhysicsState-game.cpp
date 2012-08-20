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
	dirty_verlet_islands = false;
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

	verlet_islands.clear();

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

	step();
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

	for ( Contact& ct : rigid_contacts ) game->rd.drawContact( ct );
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
	buffer << "/" << rigid_shapes.size();
	buffer << "/" << rigid_contacts.size();
	buffer << "/?"; // TODO: number of islands
	buffer << ", " << eus.size();
	buffer << ", " << vls.size() << "-" << dcs.size() << "-" << acs.size();
	buffer << "/" << verlet_islands.size();
}
