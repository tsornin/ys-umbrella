#include "PhysicsState.h"
#include <iostream>

/*
================================
PhysicsState::createRigid

Creates a new Rigid body shaped like the specified MeshOBJ.
================================
*/
Rigid* PhysicsState::createRigid( const MeshOBJ& obj )
{
	// Copy OBJ data
	std::vector < Convex > cs;
	for ( unsigned int i = 0; i < obj.faces.size(); ++i ) {
		std::vector < Vec2 > points;
		const FaceOBJ& f = obj.faces[i];
		for ( unsigned int i = 0; i < f.vis.size(); ++i ) {
			VertexOBJ v = obj.getVertex( f.vis[i] );
			points.push_back( Vec2( v.x, v.y ) * obj.scale );
		}
		cs.push_back( Convex( points ) );
	}

	Rigid* rg = new Rigid( cs );
	rg->pid = nextPID();
	rg->it = rgs.insert( rgs.end(), rg );
	return rg;
}

/*
================================
PhysicsState::createRigid
================================
*/
Rigid* PhysicsState::createRigid()
{
	Rigid* rg = new Rigid();
	rg->pid = nextPID();
	rg->it = rgs.insert( rgs.end(), rg );
	return rg;
}

/*
================================
PhysicsState::destroyRigid
================================
*/
void PhysicsState::destroyRigid( Rigid* rg )
{
	auto edges_copy = rg->edges;
	for ( Constraint* ct : edges_copy ) {
		ct->destroy( *this );
	}
	assert( rg->isolated() );

	rgs.erase( rg->it );
	delete rg;
}

/*
================================
PhysicsState::createFriction
================================
*/
Friction* PhysicsState::createFriction( Rigid* a, Rigid* b )
{
	Friction* ft = new Friction( a, b );
	ft->pid = nextPID();
	ft->it = cts.insert( cts.end(), ft );
	return ft;
}

/*
================================
PhysicsState::destroyFriction
================================
*/
void PhysicsState::destroyFriction( Friction* ft )
{
	cts.erase( ft->it );
	delete ft;
}

/*
================================
PhysicsState::createContact

Creates a new Contact constraint.
================================
*/
std::pair < bool, Contact* >
PhysicsState::createContact( Rigid* a, Rigid* b, ContactKey& key )
{
	bool cache_hit;
	Contact* ct;

	auto find = contact_cache.find( key );
	if ( find != contact_cache.end() ) {
		cache_hit = true;
		ct = find->second;
	}
	else {
		cache_hit = false;
		ct = new Contact( a, b );
		ct->pid = nextPID();
		ct->it = cts.insert( cts.end(), ct );

		// Just for destroyContact
		ct->key = key;
		contact_cache[ key ] = ct;
	}

	ct->expired = false;

	return std::pair < bool, Contact* >( cache_hit, ct );
}

/*
================================
PhysicsState::destroyContact
================================
*/
void PhysicsState::destroyContact( Contact* ct )
{
	contact_cache.erase( ct->key );

	cts.erase( ct->it );
	delete ct;
}

/*
================================
PhysicsState::createEuler

Creates a new Euler particle.
================================
*/
Euler* PhysicsState::createEuler()
{
	Euler* eu = new Euler();
	eu->pid = nextPID();
	eu->it = eus.insert( eus.end(), eu );
	return eu;
}

/*
================================
PhysicsState::destroyEuler
================================
*/
void PhysicsState::destroyEuler( Euler* eu )
{
	eus.erase( eu->it );
	delete eu;
}

/*
================================
PhysicsState::createVerlet

Creates a new Verlet particle.
================================
*/
Verlet* PhysicsState::createVerlet()
{
	dirty_verlet_islands = true;

	Verlet* vl = new Verlet();
	vl->pid = nextPID();
	vl->it = vls.insert( vls.end(), vl );
	return vl;
}

/*
================================
PhysicsState::destroyVerlet

NOTE: Verlet particles and Distance constraints form a graph.
Destroying a Verlet will also destroy all connected Distance objects.
================================
*/
void PhysicsState::destroyVerlet( Verlet* vl )
{
	dirty_verlet_islands = true;

	auto edges_copy = vl->edges;
	for ( Distance* dc : edges_copy ) {
		destroyDistance( dc );
	}
	assert( vl->isolated() );

	vls.erase( vl->it );
	delete vl;
}

/*
================================
PhysicsState::createDistance

Creates a new Distance constraint.

TODO: Verify that the specified Verlet particles aren't already constrained.
================================
*/
Distance* PhysicsState::createDistance( Verlet* a, Verlet* b )
{
	dirty_verlet_islands = true;

	Distance* dc = new Distance( a, b );
	dc->pid = nextPID();
	dc->it = dcs.insert( dcs.end(), dc );
	return dc;
}

/*
================================
PhysicsState::destroyDistance

NOTE: Distance constraints and Angular constraints form a graph.
Destroying a Distance will also destroy all connected Angular objects.
================================
*/
void PhysicsState::destroyDistance( Distance* dc )
{
	dirty_verlet_islands = true;

	auto edges_copy = dc->edges;
	for ( Angular* ac : edges_copy ) {
		destroyAngular( ac );
	}
	assert( dc->isolated() );

	dcs.erase( dc->it );
	delete dc;
}

/*
================================
PhysicsState::createAngular

Creates a new Angular constraint.

Returns null unless the given Distance constraints
share endpoints "m->b" and "n->a".

TODO: Verify that the specified Distance constraints aren't already constrained.
================================
*/
Angular* PhysicsState::createAngular( Distance* m, Distance* n )
{
	return 0; // TODO: operation currently not supported

/*
	dirty_verlet_islands = true;

	if ( m->b != n->a ) return 0;

	Angular* ac = new Angular( m, n );
	ac->pid = nextPID();
	acs.push_back( ac );

	// Here are the three masses we'll be clamping
	Verlet* a = ac->a = m->a;
	Verlet* b = ac->b = m->b; // n->a
	Verlet* c = ac->c = n->b;

	// TODO: This isn't terribly good positioning
	Vec2 left = ( c->getPosition() - a->getPosition() ).lperp() * 0.25;
	// TODO: Is this the best we can do for mass?
	Scalar mass = b->getMass();
	Vec2 gravity = b->getGravity();

	// Supporting Verlet masses
	Verlet* l = ac->l = createVerlet();
	l->putPosition( b->getPosition() + left );
	l->setMass( mass );
	l->setGravity( gravity );

	Verlet* r = ac->r = createVerlet();
	r->putPosition( b->getPosition() - left );
	r->setMass( mass );
	r->setGravity( gravity );

	// Supporting Distance constraints
	ac->al = createDistance( a, l ); ac->ar = createDistance( a, r );
	ac->bl = createDistance( b, l ); ac->br = createDistance( b, r );
	ac->cl = createDistance( c, l ); ac->cr = createDistance( c, r );
	ac->h = createDistance( l, r );
	ac->v = createDistance( a, c );

	// Graph setup
	ac->m->edges.insert( ac );
	ac->n->edges.insert( ac );

	return ac;
*/
}

/*
================================
PhysicsState::destroyAngular
================================
*/
void PhysicsState::destroyAngular( Angular* ac )
{
	// TODO: operation currently not supported
	return;

/*
	dirty_verlet_islands = true;

	// No higher destroy calls
	// (Angular is at the top of the dual-graph food chain)

	// Destroy our physics objects.
	// Destroying the two supporting Verlet masses
	// removes all constraints except the vertical one.
	destroyVerlet( ac->l );
	destroyVerlet( ac->r );
	destroyDistance( ac->v );

	// Vertex edge-list removal
	// (this is why destroyDistance can't iterate edges)
	ac->m->edges.erase( ac );
	ac->n->edges.erase( ac );

	// Flag for deletion
	ac->expire_enable = true;
*/
}

/*
================================
PhysicsState::nearestVerlet

Returns the Verlet particle nearest to the specified location.
Returns null if there are no Verlet particles within the specified radius.

TODO: Make this faster?
================================
*/
Verlet* PhysicsState::nearestVerlet( const Vec2& p, Scalar r )
{
	Verlet* ret = 0;
	Scalar score = SCALAR_MAX;

	for ( Verlet* vl : vls ) {
		Scalar rr = (vl->position - p).length2();
		if ( rr < score ) {
			ret = vl;
			score = rr;
		}
	}

	return score < r*r ? ret : 0;
}

Rigid* PhysicsState::nearestRigid( const Vec2& p )
{
	for ( auto pair : rigid_shapes ) {
		Rigid* rg = pair.first.first;
		Convex& c = pair.second;
		if ( rg->frozen() ) continue;
		if ( c.contains( p ) ) return rg;
	}

	return 0;
}
