#include "Euler.h"
#include "Constants.h"
#include "spatial/AABB.h"
#include "game/InputSet.h"

/*
================================
Euler::Euler
================================
*/
Euler::Euler() :
	// Position state
	position( 0, 0 ), velocity( 0, 0 ),
		linear_enable( true ),
	// Damping
	linear_damping( STANDARD_LINEAR_DAMPING ),
	// Gravity
	gravity( 0 ),
	// Collision properties
	mass( STANDARD_MASS ),
	bounce( STANDARD_BOUNCE )
{
	
}

/*
================================
Euler::input

Provides general-purpose movement and rotation in any direction.
NOTE: Diagonal speed is bugged ("strafe bug") to be faster.
================================
*/
void Euler::input( const InputSet& is )
{
	// TODO: Totally arbitrary
	Scalar acc = GRAVITY_HI / 2;

	// Speed modifiers
	Scalar multiplier = 1.0;
	bool lo = is.get( IK_SL );
	bool hi = is.get( IK_SR );
	if ( hi != lo ) {
		if ( lo ) multiplier = 0.25;
		if ( hi ) multiplier = 4.0;
	}
	acc *= multiplier;

	// Freeze toggling
	if ( is.rising( IK_A ) ) linear_enable = !linear_enable;

	// Linear movement
	if ( linear_enable ) {
		if ( is.get( IK_U ) )	velocity.y += acc;
		if ( is.get( IK_D ) )	velocity.y -= acc;
		if ( is.get( IK_L ) )	velocity.x -= acc;
		if ( is.get( IK_R ) )	velocity.x += acc;
	}
}

/*
================================
Euler::update
================================
*/
void Euler::update()
{
	if ( linear_enable ) {
		position += velocity;
	}
	else {
		velocity = Vec2( 0, 0 );
	}
}

/*
================================
Euler::getAABB
================================
*/
AABB Euler::getAABB() const
{
	return AABB( position );
}
