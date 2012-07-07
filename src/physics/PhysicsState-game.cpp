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
	for ( TEuler& te : eus ) delete te.second;
	eus.clear();

	// for ( TRigid& tr : rgs ) delete tr.second;
	// rgs.clear();

	for ( TVerlet& tv : vls ) delete tv.second;
	vls.clear();

	// for ( TDistance& td : dcs ) delete td.second;
	// dcs.clear();

	// for ( Angular* ac : acs ) delete ac;
	// acs.clear();

	// connected_components.clear();

	// clear_collision_data();

	// hr.cleanup();

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

	// // Identify CC first,
	// // so Verlets are tagged with their component ID
	// if ( dirty_connected_components ) {
	// 	find_connected_components();
	// 	dirty_connected_components = false;
	// }

	// // During RG-VL detection,
	// // wall constraints are tagged with Verlet component IDs
	// detect_collisions();

	// relax_connected_components();
}

/*
================================
PhysicsState::draw
================================
*/
void PhysicsState::draw( Engine* game )
{
	// hr.drawAxes( 100 );

	// // Display the bounding boxes of the convex list.
	// // (The AABB's of Rigid::shapes are in object space, which isn't useful)
	// for ( auto& pair : vxs ) {
	// 	Convex& vx = pair.second;
	// 	hr.drawAABB( vx.getAABB() );
	// }

	// for ( TRigid& tr : rgs ) {
	// 	hr.drawRigid( *tr.second, rt_colors[ tr.first ] );
	// }

	// for ( TEuler& te : eus ) {
	// 	hr.drawEuler( *te.second, et_colors[ te.first ] );
	// }

	// for ( TDistance& td : dcs ) {
	// 	hr.drawDistance( *td.second, dt_colors[ td.first ] );
	// }

	// // TODO: get rid of this
	// // for ( Angular* ac : acs ) {
	// 	// hr.drawAngular( *ac, 0 ); // TODO: color
	// // }

	// for ( TVerlet& tv : vls ) {
	// 	hr.drawVerlet( *tv.second, vt_colors[ tv.first ] );
	// }
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

	// buffer << " " << eus.size() << "/" << rgs.size() << "/" << vls.size();
	// buffer << " " << dcs.size() << "/" << acs.size();
	// buffer << " cc: " << connected_components.size();
}
