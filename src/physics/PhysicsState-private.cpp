#include "PhysicsState.h"
#include <queue>
#include <algorithm> // for std::remove_if

/*
================================
Expire

Functor for std::remove_if,
called from PhysicsState::expire
================================
*/

template < typename T >
struct Expire {
	bool operator() ( T& t ) {
		if ( t->expired() ) {
			delete t;
			return true;
		}
		return false;
	}
};

/*
================================
PhysicsState::expire

Removes and deletes expired physics objects.
================================
*/
void PhysicsState::expire()
{
	// This invalidates old indices.
	rgs.erase( std::remove_if( rgs.begin(), rgs.end(), Expire < Rigid* >() ), rgs.end() );
	eus.remove_if( Expire < Euler* >() );
	vls.remove_if( Expire < Verlet* >() );
	dcs.remove_if( Expire < Distance* >() );
	acs.remove_if( Expire < Angular* >() );
}

/*
================================
PhysicsState::integrate

Applies forces to velocity and velocity to position, creating collisions.
================================
*/
void PhysicsState::integrate()
{
	integrate_velocity();
	integrate_position();
}

/*
================================
PhysicsState::integrate_velocity

Applies forces (gravity, wind) to velocities
in preparation for the position update.
================================
*/
void PhysicsState::integrate_velocity()
{
	apply_gravity_forces();
	apply_wind_forces();
}

/*
================================
PhysicsState::apply_gravity_forces

TODO:
design a gravity system
	0. global gravity
	1. per-particle gravity
	2. getGravityAt - gravity regions
stop manually applying gravity at Entity level
================================
*/
void PhysicsState::apply_gravity_forces()
{
	// TODO: we probably don't want to apply gravity here,
	// since the Lambda matrix is computed with V and F_ext.
	// Instead, we should add to the F_ext matrix.
	// for ( Rigid* rg : rgs ) {
	// 	rg->addVelocity( rg->gravity );
	// }

	for ( Euler* eu : eus ) eu->addVelocity( eu->gravity );

	for ( Verlet* vl : vls ) vl->addPosition( vl->gravity );
}

/*
================================
PhysicsState::apply_wind_forces

TODO:
design a wind system
	global drag
	vector wind
	fluid wind (coarse realtime)
	fluid wind (fine precomputed "aura)
remove stokes resistance at physics object level
================================
*/
void PhysicsState::apply_wind_forces()
{
	// Apply wind to bodies using PhysicsState::getWindAt.
	// for ( TEuler& te : eus ) {
		// EulerType et = te.first;
		// Euler* eu = te.second;
		// // TODO: top emitters aren't affected, etc.
		// if ( te.second TYPE IS AFFECTED BY WIND ) {
			// eu->velocity = Vec2::interp(
				// eu->velocity,
				// getWindAt( eu->position ),
				// eu->linear_damping );
		// }
	// }

	// TODO: Wind forces to Rigid are a bit different.
	// 1. Add forces to F_ext
	// 2. Add forces per projected Convex?
}

/*
================================
PhysicsState::integrate_position

Applies velocities to positions, possibly creating collisions.
================================
*/
void PhysicsState::integrate_position()
{
	for ( Rigid* rg : rgs ) rg->update();
	for ( Euler* eu : eus ) eu->update();
	for ( Verlet* vl : vls ) vl->update();
}

/*
================================
PhysicsState::find_connected_components

Finds all connected components of the
Verlet particle / Distance constraint graph.

Invariant: no Verlet particles are marked
================================
*/
void PhysicsState::find_connected_components()
{
	connected_components.clear();

	// Undirected connected components algorithm
	for ( Verlet* vl : vls ) {
		if ( vl->marked ) continue;

		VerletGraph vg = mark_connected( vl );

		// Tag every Verlet with the component ID
		int id = connected_components.size();
		for ( Verlet* vl : vg.first ) {
			vl->component_id = id;
		}

		connected_components.push_back( vg );
	}

	// ASSERT: All vertices should be marked
	// for ( Verlet* vl : vls ) {
		// assert( vl->marked );
	// }

	// Post-condition
	for ( Verlet* vl : vls ) {
		vl->marked = false;
	}
}

/*
================================
PhysicsState::relax_connected_components

Applies relaxation to each connected component of Verlet particles.

Repeatedly applies distance constraints between Verlet particles,
converging toward the solution and implicitly modifying velocity.
================================
*/
void PhysicsState::relax_connected_components()
{
	for ( VerletGraph vg : connected_components ) {
		// std::vector < Verlet* >& vls = vg.first;
		std::vector < Distance* >& dcs = vg.second;

		// Estimate the number of iterations needed
		int m = (int) std::ceil( std::sqrt( dcs.size() ) );

		// Relax distance constraints with wall contacts
		for ( int i = 0; i < m; ++i ) {
			for ( Distance* dc : dcs ) {
				dc->apply();
			}

			// TODO: equal_range can be moved outside one for block.
			// Every component has at least one vertex
			// int id = vls.front()->component_id;
			// auto range = verlet_wall_contacts.equal_range( id );
			// for ( auto it = range.first; it != range.second; ++it ) {
			// 	// Seems like this works best with biased Convex::contains
			// 	WallContact& wc = it->second;
			// 	Verlet* vl = wc.first;
			// 	Wall& w = wc.second;

			// 	auto ret = w.contains( vl->getPosition() );
			// 	if ( ret.first ) vl->addPosition( ret.second );
			// }
		}
	}
}

/*
================================
PhysicsState::mark_connected

Returns all Verlet particles and Distance constraints in
the connected component of the specified Verlet particle.

Post-condition: all Verlet particles returned are marked
================================
*/
VerletGraph PhysicsState::mark_connected( Verlet* root )
{
	VerletGraph ret;
	std::vector < Verlet* >& vls = ret.first;
	std::vector < Distance* >& dcs = ret.second;

	// Start the BFS
	std::queue < Verlet*, std::list < Verlet* > > unseen;
	unseen.push( root );
	vls.push_back( root );
	root->marked = true;

	while ( ! unseen.empty() ) {
		Verlet* v = unseen.front();
		unseen.pop();

		for ( Distance* dc : v->edges ) {
			// assert( dc->a == v || dc->b == v );
			Verlet* w = ( dc->a == v ) ? dc->b : dc->a;
			if ( ! w->marked ) {
				unseen.push( w );
				vls.push_back( w );
				w->marked = true;
			}

			// This finds all edges twice
			dcs.push_back( dc );
		}
	}

	// Remove duplicate edges
	std::sort( dcs.begin(), dcs.end() );
	dcs.erase( std::unique( dcs.begin(), dcs.end() ), dcs.end() );

	// for ( Verlet* vl : vls ) {
		// assert( vl->marked );
	// }

	return ret;
}

/*
================================
PhysicsState::detect_collisions
================================
*/
void PhysicsState::detect_collisions()
{
	clear_collision_data();
	transform_convex();

	// detect_solve_euler_rigid();
	// detect_solve_verlet_rigid();
}

/*
================================
PhysicsState::clear_collision_data

Clears all collision data from the last frame.
================================
*/
void PhysicsState::clear_collision_data()
{
	// pgs.clear();
	// verlet_wall_contacts.clear();
}

/*
================================
PhysicsState::transform_convex

Makes copies of all Rigid-owned Convex shapes,
transformed into world space and tagged with owners.
This happens every frame; there are no deletion problems.

TODO: A more sophisticated on-demand transforming scheme?
================================
*/
void PhysicsState::transform_convex()
{
	// for ( TRigid& tr : rgs ) {
	// 	Rigid* rg = tr.second;
	// 	for ( const Convex& pg : rg->shapes ) {
	// 		Convex t( pg );
	// 		t.transform( rg->position, rg->angle );
	// 		pgs.push_back( std:pair < TRigid, Convex >( tr, t ) );
	// 	}
	// }
}

/*
================================
PhysicsState::nextPID
================================
*/
int PhysicsState::nextPID()
{
	return next_pid++;
}
