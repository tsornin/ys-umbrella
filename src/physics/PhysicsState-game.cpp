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

	dirty_verlet_islands = false;
}

/*
================================
PhysicsState::cleanup

NOTE: Clearing the lists isn't voodoo (PhysicsState is never destructed).
================================
*/
void PhysicsState::cleanup()
{
	for ( Rigid* rg : rgs ) delete rg;
	rgs.clear();

	for ( Contact* ct : contacts ) delete ct;
	contacts.clear();

	for ( Constraint* ct : cts ) delete ct;
	cts.clear();

	rigid_islands.clear();

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
	for ( Contact* ct : contacts ) ct->draw( game->rd ); // Contact contains Friction
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
PhysicsState::setCaption
================================
*/
void PhysicsState::setCaption( std::ostringstream& buffer )
{
	buffer << " || Physics:";
	buffer << " " << frames_elapsed << " frames elapsed";

	buffer << " " << rgs.size();
	buffer << " (" << rigid_shapes.size() << ")";
	buffer << "-" << contacts.size();
	buffer << "-" << cts.size();
	buffer << "/" << rigid_islands.size();

	for ( RigidGraph& rgg : rigid_islands ) {
		buffer << " {" << rgg.first.size() << "-" << rgg.second.size() << "}";
		buffer << " pid:[";
		for ( Rigid* rg : rgg.first ) {
			buffer << " " << rg->pid;
		}
		buffer << " ]";
	}

	buffer << ", " << eus.size();

	buffer << ", " << vls.size() << "-" << dcs.size() << "-" << acs.size();
	buffer << "/" << verlet_islands.size();

	buffer << ", next_pid: " << next_pid;
}
