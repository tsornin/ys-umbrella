#include "PhysicsState.h"
#include "Euler.h"
#include "Verlet.h"

/*
================================
Expire

Functor for std::list < T* >::remove_if,
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
TExpire

Functor for std::list < T* >::remove_if,
called from PhysicsState::expire
================================
*/

template < typename T >
struct TExpire {
	bool operator() ( T& t ) {
		if ( t.second->expired() ) {
			delete t.second;
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
	eus.remove_if( TExpire < TEuler >() );
	// rgs.remove_if( TExpire < TRigid >() );
	vls.remove_if( TExpire < TVerlet >() );
	// dcs.remove_if( TExpire < TDistance >() );

	// acs.remove_if( Expire < Angular* >() );
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
	for ( TEuler& te : eus ) {
		Euler* eu = te.second;
		eu->addVelocity( eu->gravity );
	}

	// TODO: rigid...

	for ( TVerlet& tv : vls ) {
		Verlet* vl = tv.second;
		vl->addPosition( vl->gravity );
	}
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
}

/*
================================
PhysicsState::integrate_position

Applies velocities to positions, possibly creating collisions.
================================
*/
void PhysicsState::integrate_position()
{
	for ( TEuler& te : eus ) te.second->update();
	// for ( TRigid& tr : rgs ) tr.second->update();
	for ( TVerlet& tv : vls ) tv.second->update();
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
