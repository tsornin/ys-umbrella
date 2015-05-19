#include "PhysicsState.h"
#include "spatial/AABB.h"

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

	// anchor = new Rigid();
	// anchor->pid = nextPID();
	// anchor->mask = 0;
	// anchor->setLinearEnable( false );
	// anchor->setAngularEnable( false );
	// rgs.push_back( anchor );
}

/*
================================
PhysicsState::cleanup

NOTE: Clearing the lists isn't voodoo (PhysicsState is never destructed).
================================
*/
void PhysicsState::cleanup()
{
	auto rgs_copy = rgs;
	for ( Rigid* rg : rgs_copy ) destroyRigid( rg );
	assert( rgs.empty() );
	assert( cts.empty() );

	assert( contact_cache.empty() );

	auto eus_copy = eus;
	for ( Euler* eu : eus_copy ) destroyEuler( eu );
	assert( eus.empty() );

	// TODO: make non-user Verlet filtering less ugly
	std::list < Verlet* > user_vls;
	for ( Verlet* vl : vls ) {
		if ( vl->pid < 0 ) continue;
		user_vls.push_back( vl );
	}
	for ( Verlet* vl : user_vls ) {
		destroyVerlet( vl );
	}
	assert( vls.empty() );
	assert( dcs.empty() );
	assert( acs.empty() );

	clear_collision_data();

	BlankState::cleanup();

	next_pid = 0;
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
	// 	Convex& c = pair.second;
	// 	game->rd.drawAABB( c.getAABB() );
	// }

	for ( Rigid* rg : rgs ) game->rd.drawRigid( *rg );
	for ( Constraint* ct : cts ) ct->draw( game->rd ); // Constraint is polymorphic

	for ( Euler* eu : eus ) game->rd.drawEuler( *eu );

	for ( Distance* dc : dcs ) game->rd.drawDistance( *dc );
	// for ( Angular* ac : acs ) game->rd.drawAngular( *ac );
	for ( Verlet* vl : vls ) game->rd.drawVerlet( *vl );


	// // TODO: Display islands.
	// // TODO: don't do this. also, get rid of #include AABB
	// for ( VerletGraph& vlg : verlet_islands ) {
	// 	AABB box = vlg.first.front()->getAABB();
	// 	for ( Verlet* vl : vlg.first ) {
	// 		box += vl->getAABB();
	// 	}
	// 	game->rd.drawAABB( box );
	// }

	// // TODO: remember, Rigid::getAABB incurs extra transforms
	// for ( RigidGraph& rgg : rigid_islands ) {
	// 	AABB box = rgg.first.front()->getAABB();
	// 	for ( Rigid* rg : rgg.first ) {
	// 		box += rg->getAABB();
	// 	}
	// 	game->rd.drawAABB( box );
	// }
}

/*
================================
PhysicsState::frame_caption
================================
*/
void PhysicsState::frame_caption( std::ostringstream& buffer )
{
	buffer << " || Physics:";
	buffer << " " << frames_elapsed << " frames elapsed";
}

/*
================================
PhysicsState::frame_printout
================================
*/
void PhysicsState::frame_printout( std::ostringstream& buffer )
{
	buffer << "\n" << "--------------------------------";
	buffer << "\n" << "Physics frame: " << frames_elapsed;

	buffer << "\n" << "Rigid:"
		<< "\n\t" << rgs.size() << " rigid bodies"
		<< "\n\t" << rigid_shapes.size() << " shapes"
		<< "\n\t" << contact_cache.size() << " contacts cached"
		<< "\n\t" << cts.size() << " contacts"
		<< "\n\t" "across " << rigid_islands.size() << " islands";

	buffer << "\n" << "Euler:"
		<< "\n\t" << eus.size() << " euler particles";

	buffer << "\n" << "Verlet:"
		<< "\n\t" << vls.size() << " verlet particles"
		<< "\n\t" << dcs.size() << " distance constraints"
		<< "\n\t" << acs.size() << " angular constraints"
		<< "\n\t" << "across " << verlet_islands.size() << " islands";

	buffer << "\n" << "next_pid: " << next_pid;
}
