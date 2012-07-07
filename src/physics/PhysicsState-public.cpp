#include "PhysicsState.h"
#include "Euler.h"
#include "Verlet.h"
#include "Distance.h"
#include "Angular.h"

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
	vl->marked = false;
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
	while ( !vl->edges.empty() ) {
		destroyDistance( *(vl->edges.begin()) );
	}

	// No vertex edge-list removal
	// (Verlet is the bottom of the dual-graph food chain)

	// Flag for deletion
	vl->expire_enable = true;
}

/*
================================
PhysicsState::createDistance

Creates a new Distance constraint.

TODO: Verify that the specified Verlet particles aren't already constrained.
================================
*/
Distance* PhysicsState::createDistance( Verlet* a, Verlet* b, DistanceType dt )
{
	dirty_connected_components = true;

	Distance* dc = new Distance( a, b );
	dc->pid = nextPID();
	dcs.push_back( TDistance( dt, dc ) );

	dc->a->edges.insert( dc );
	dc->b->edges.insert( dc );

	return dc;
}

/*
================================
PhysicsState::destroyDistance

Marks the specified Distance constraint for deletion.

NOTE: Distance constraints and Angular constraints form a graph.
Destroying a Distance will also destroy all connected Angular objects.
================================
*/
void PhysicsState::destroyDistance( Distance* dc )
{
	dirty_connected_components = true;

	// Call destroyAngular on all our edges.
	// We can't iterate our edges, though,
	// since destroyAngular deletes them.
	while ( !dc->edges.empty() ) {
		destroyAngular( *(dc->edges.begin()) );
	}

	// Vertex edge-list removal
	// (this is why destroyVerlet can't iterate edges)
	dc->a->edges.erase( dc );
	dc->b->edges.erase( dc );

	// Flag for deletion
	dc->expire_enable = true;
}

/*
================================
PhysicsState::createAngular

Creates a new Angular constraint.

Returns null unless the given Distance constraints
share endpoints "m->b" and "n->a".

TODO: Verify that the specified Distnace constraints aren't already constrained.
================================
*/
Angular* PhysicsState::createAngular( Distance* m, Distance* n )
{
	dirty_connected_components = true;

	if ( m->b != n->a ) return 0;

	Angular* ac = new Angular( m, n );
	ac->pid = nextPID();
	acs.push_back( ac );

	Verlet* a = ac->a = m->a;
	Verlet* b = ac->b = m->b; // n->a
	Verlet* c = ac->c = n->b;

	// TODO: This isn't terribly good positioning
	Vec2 left = ( c->getPosition() - a->getPosition() ).lperp() * 0.25;
	// TODO: Is this the best we can do for mass?
	Scalar mass = b->getMass();

	Verlet* l = ac->l = createVerlet( (VerletType)0 );
	l->putPosition( b->getPosition() + left );
	l->setMass( mass );

	Verlet* r = ac->r = createVerlet( (VerletType)0 );
	r->putPosition( b->getPosition() - left );
	r->setMass( mass );

	ac->al = createDistance( a, l, (DistanceType)0 ); ac->ar = createDistance( a, r, (DistanceType)0 );
	ac->bl = createDistance( b, l, (DistanceType)0 ); ac->br = createDistance( b, r, (DistanceType)0 );
	ac->cl = createDistance( c, l, (DistanceType)0 ); ac->cr = createDistance( c, r, (DistanceType)0 );
	ac->h = createDistance( l, r, (DistanceType)0 );
	ac->v = createDistance( a, c, (DistanceType)0 );

	ac->m->edges.insert( ac );
	ac->n->edges.insert( ac );

	return ac;
}

/*
================================
PhysicsState::destroyAngular
================================
*/
void PhysicsState::destroyAngular( Angular* ac )
{
	dirty_connected_components = true;

	// No higher destroy calls
	// (Angular is at the top of the dual-graph food chain)

	// Destroy our physics objects.
	// Destroying the two Verlet supports removes all constraints except ac->v.
	destroyVerlet( ac->l );
	destroyVerlet( ac->r );
	destroyDistance( ac->v );

	// Vertex edge-list removal
	// (this is why destroyDistance can't iterate edges)
	ac->m->edges.erase( ac );
	ac->n->edges.erase( ac );

	// Flag for deletion
	ac->expire_enable = true;
}

/*
================================
PhysicsState::connected

Returns all Verlet particles and Distance constraints in
the connected component of the specified Verlet particle.

Invariant: no Verlet particles are marked
================================
*/
VerletGraph PhysicsState::connected( Verlet* root )
{
	VerletGraph ret = mark_connected( root );

	// Unmark everything we found
	for ( Verlet* vl : ret.first ) {
		vl->marked = false;
	}

	return ret;
}
