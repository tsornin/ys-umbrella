#include "Rigid.h"
#include "Constants.h"
#include "spatial/AABB.h"
#include "game/InputSet.h"

/*
================================
Rigid::Rigid
================================
*/
Rigid::Rigid() :
	// Position state
	position( 0, 0 ), velocity( 0, 0 ),
		linear_enable( true ),
	// Rotation state
	angular_position( 0 ), angular_velocity( 0 ),
		angular_enable( true ),
	// Damping
	linear_damping( STANDARD_LINEAR_DAMPING ),
	angular_damping( STANDARD_ANGULAR_DAMPING ),
	// Gravity
	gravity( 0 ),
	// Collision properties
	mass( STANDARD_MASS ),
	moment( STANDARD_MOMENT ),
	bounce( STANDARD_BOUNCE ),
	friction( STANDARD_FRICTION ),
	// Physics engine graph data
	marked( false ),
	local_id( -1 )
{
	
}

/*
================================
Rigid::Rigid

Creates a Rigid body with mass properties
computed from the specified Convex shapes.
================================
*/
Rigid::Rigid( const std::vector < Convex >& pgs ) :
	Rigid()
{
	shapes = pgs;

	int n = shapes.size();

	// Compute convex properties
	// Save data for aggregate calculations
	std::vector < Scalar > masses(n);
	std::vector < Scalar > moments(n);
	std::vector < Vec2 > centroids(n);
	for ( int i = 0; i < n; ++i ) {
		Scalar mass; Scalar moment; Vec2 centroid;
		shapes[i].calculate( mass, moment, centroid );

		masses[i] = ( mass );
		moments[i] = ( moment );
		centroids[i] = ( centroid );
	}

	// Compute aggregate properties
	// Avoid having zero-mass rigid bodies.
	mass = STANDARD_MASS;
	moment = STANDARD_MOMENT;

	// Sum masses first
	for ( int i = 0; i < n; ++i ) {
		mass += masses[i];
	}

	// Find centroid with mass-weighted average of position
	for ( int i = 0; i < n; ++i ) {
		position += centroids[i] * masses[i];
	}
	position /= mass;

	// Now that we have the centroid (which is the Rigid's position),
	// we can sum the moments (calculated about individual Convex centroids)
	// using the parallel axis theorem.
	for ( int i = 0; i < n; ++i ) {
		Vec2 r = centroids[i] - position;
		moment += moments[i] + masses[i] * (r*r);
	}

	// Now that we have the centroid, we shift shapes to object space
	for ( int i = 0; i < n; ++i ) {
		shapes[i].translate( -position );
	}
}

/*
================================
Rigid::input

Provides general-purpose movement and rotation in any direction.
NOTE: Diagonal speed is bugged ("strafe bug") to be faster.
================================
*/
void Rigid::input( const InputSet& is )
{
	// TODO: Totally arbitrary
	Scalar acc = GRAVITY_HI / 2;
	Scalar ang_acc = 0.01;
	
	// Speed modifiers
	Scalar multiplier = 1.0;
	bool lo = is.get( IK_SL );
	bool hi = is.get( IK_SR );
	if ( hi != lo ) {
		if ( lo ) multiplier = 0.25;
		if ( hi ) multiplier = 4.0;
	}
	acc *= multiplier;
	ang_acc *= multiplier;
	
	// Freeze toggling
	if ( is.rising( IK_A ) ) linear_enable = !linear_enable;
	if ( is.rising( IK_B ) ) angular_enable = !angular_enable;
	
	// Linear movement
	if ( linear_enable ) {
		if ( is.get( IK_U ) )	velocity.y += acc;
		if ( is.get( IK_D ) )	velocity.y -= acc;
		if ( is.get( IK_L ) )	velocity.x -= acc;
		if ( is.get( IK_R ) )	velocity.x += acc;
	}
	
	// Angular movement
	if ( angular_enable ) {
		if ( is.get( IK_Y ) )	angular_velocity -= ang_acc;
		if ( is.get( IK_X ) )	angular_velocity += ang_acc;
	}
}

/*
================================
Rigid::update
================================
*/
void Rigid::update()
{
	if ( linear_enable ) {
		position += velocity;
	}
	else {
		velocity = Vec2( 0, 0 );
	}

	if ( angular_enable ) {
		angular_position += angular_velocity;
		std::fmod( angular_position, 2*PI );
	}
	else {
		angular_velocity = 0;
	}
}

/*
================================
Rigid::getAABB

Returns a bounding box containing all the bounding boxes
of the Convex shapes that make up this Rigid, in world space.

Expensive. In the process,
the world transform of each shape is computed, then discarded.
================================
*/
AABB Rigid::getAABB() const
{
	// Start with a box enclosing the centroid.
	// TODO: The centroid may not always be inside the shapes-box,
	// (although it is for "well-formed" bodies such as those from
	// PhysicsState::createRigid).
	AABB box( position );

	// TODO (convex concept):
	// The plus operator is planned to become the Minkowski Sum.
	int n = shapes.size();
	for ( int i = 0; i < n; ++i ) {
		Convex pg( shapes[i] );
		pg.transform( position, angular_position );
		box += pg.getAABB();
	}
	return box;
}
