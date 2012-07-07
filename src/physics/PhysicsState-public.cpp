#include "PhysicsState.h"
#include "Euler.h"
#include "Verlet.h"

/*
================================
PhysicsState::createVerlet

Creates a new Verlet particle.
================================
*/
Verlet* PhysicsState::createVerlet( VerletType vt )
{
	dirty_connected_components = true;

	Verlet* vl = new Verlet();
	vl->pid = nextPID();
	// vl->marked = false;
	vls.push_back( TVerlet( vt, vl ) );
	return vl;
}

/*
================================
PhysicsState::destroyVerlet

Marks the specified Verlet particle for deletion.

NOTE: Verlet particles and Distance constraints form a graph.
Destroying a Verlet will also destroy all connected Distance objects.
================================
*/
void PhysicsState::destroyVerlet( Verlet* vl )
{
	dirty_connected_components = true;

	// Call destroyDistance on all our edges.
	// We can't iterate our edges, though,
	// since destroyDistance deletes them.
	// while ( !vl->edges.empty() ) {
	// 	destroyDistance( *(vl->edges.begin()) );
	// }

	// No vertex edge-list removal
	// (Verlet is the bottom of the dual-graph food chain)

	// Flag for deletion
	vl->expire_enable = true;
}

/*
================================
PhysicsState::createEuler

Creates a new Euler particle.
================================
*/
Euler* PhysicsState::createEuler( EulerType et )
{
	Euler* eu = new Euler();
	eu->pid = nextPID();
	eus.push_back( TEuler( et, eu ) );
	return eu;
}

/*
================================
PhysicsState::destroyEuler

Marks the specified Euler particle for deletion.
================================
*/
void PhysicsState::destroyEuler( Euler* eu )
{
	eu->expire_enable = true;
}
