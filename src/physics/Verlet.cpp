#include "Verlet.h"
#include "Constants.h"
#include "spatial/AABB.h"

/*
================================
Verlet::Verlet
================================
*/
Verlet::Verlet() :
	// Position state
	position( 0, 0 ), velocity( 0, 0 ), previous( 0, 0 ),
		linear_enable( true ),
	// Damping
	linear_damping( STANDARD_LINEAR_DAMPING ),
	// Gravity
	gravity( 0 ),
	// Collision properties
	mass( STANDARD_MASS ),
	bounce( 0 ),
	// // Physics engine graph data
	marked( false ),
	component_id( -1 )
{
	
}

/*
================================
Verlet::update

Fixed timestep Verlet integration with damping.
================================
*/
void Verlet::update()
{
	if ( linear_enable ) {
		velocity = position - previous;
		previous = position;
		position += velocity * linear_damping; // wind resistance
	}
	else {
		previous = position;
		velocity = position - previous;
	}
}

/*
================================
Verlet::getAABB
================================
*/
AABB Verlet::getAABB() const
{
	return AABB( position );
}

/*
================================
Verlet::putPosition

Moves this particle to the specified location
and zeroes its implicit velocity.
================================
*/
void Verlet::putPosition( const Vec2& pos )
{
	position = pos;
	previous = pos;
}
