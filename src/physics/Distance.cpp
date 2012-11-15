#include "Distance.h"
#include "Angular.h" // for superclass PhysicsGraph < Distance, Angular >::Vertex
#include "spatial/Vec2.h"
#include "spatial/AABB.h"
#include "Verlet.h"

/*
================================
Distance::Distance
================================
*/
Distance::Distance( Verlet* a, Verlet* b ) :
	PhysicsGraph < Verlet, Distance >::Edge( a, b ),
	// Constraint properties
	power( 1.0 ),
	type( DC_HARD )
{
	rest_length = (b->position - a->position).length();
}

/*
================================
Distance::apply

See "Advanced Character Physics" (2003) by Thomas Jakobsen:
http://www.gamasutra.com/resource_guide/20030121/jacobson_01.shtml
================================
*/
void Distance::apply()
{
	Vec2 delta = b->position - a->position;

	// Compute delta.length(), but use
	// Newton-Raphson since "rest_length" is a good initial guess
	Scalar delta_length = rest_length;
	Scalar mag2 = delta.length2();
	delta_length = ( delta_length + mag2/delta_length ) * 0.5;
	delta_length = ( delta_length + mag2/delta_length ) * 0.5;

	// Type check
	if ( type == DC_PULL ) if ( delta_length < rest_length ) return;
	if ( type == DC_PUSH ) if ( delta_length > rest_length ) return;

	// NOTE: At this point, we have the "stretch ratio".
	// This isn't an appropriate place to break the constraint, though:
	// 1. We're still solving all constraints;
	// 2. Physics objects never delete themselves.
	// The owner of this physics object should inspect a Distance itself
	// if it wants to remove unreasonably stretched constraints.

	// Correction
	Scalar diff = ( delta_length - rest_length ) / ( delta_length );
	diff *= power;
	// Apply correction weighted by inverse mass.
	diff /= a->mass + b->mass;
	a->addPosition( delta * (  diff * b->mass ) );
	b->addPosition( delta * ( -diff * a->mass ) );
}

/*
================================
Distance::getAABB
================================
*/
AABB Distance::getAABB() const
{
	return a->getAABB() + b->getAABB();
}
