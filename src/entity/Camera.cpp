#include "Camera.h"

/*
================================
Camera response constants.
Experimentally confirmed to be aesthetically pleasing.
Assuming wide-screen: Y-response is higher that of X-response.
================================
*/
const Scalar CAMERA_RESPONSE_X = 0.04;
const Scalar CAMERA_RESPONSE_Y = 0.06;
const Scalar CAMERA_RESPONSE_Z = 0.08;

/*
================================
zoom_distance

With a FOV of 51.13, or 2 * arctan( 0.5 ),
the vertical distance visible is equal to the zoom distance.
6, 14, 38, 78, 134 blocks visible in Y direction.
9, 21, 57, 117, 201 blocks visible in X direction (aspect = 1.5).
TODO: assumed and hard-coded block size
================================
*/
static double zoom_distance( int zoom ) {
	return 50.0 * ( 6 + 8 * zoom*zoom );
}

/*
================================
Camera::Camera
================================
*/
Camera::Camera( EntityState& em ) : Entity( em ),
	eu( es.createEuler( 0 ) ),
	// TODO: magic numbers
	fov_y( 51.13 ),
	aspect( 1.5 ),
	z_near( 100.0 ),
	z_far( 20000.0 ),
	zoom( 1 ),
	z( zoom_distance( zoom ) ),
	dz( 0 )
{
	// This is pointless, since this Euler has no collision.
	eu->setOwner( this );
}

/*
================================
Camera::~Camera
================================
*/
Camera::~Camera()
{
	es.destroyEuler( eu );
}

/*
================================
Camera::input
================================
*/

// Moves the front element of this list to the back.
template < typename T >
static void cycle_front( std::list < T >& xs ) {
	if ( xs.empty() ) return;

	xs.push_back( xs.front() );
	xs.pop_front();
}

// Moves the back element of this list to the front.
template < typename T >
static void cycle_back( std::list < T >& xs ) {
	if ( xs.empty() ) return;

	xs.push_front( xs.back() );
	xs.pop_back();
}

void Camera::input( const InputSet& is )
{
	// Cycle target list
	if ( is.rising( IK_SR ) ) cycle_front( targets );
	if ( is.rising( IK_SL ) ) cycle_back( targets );

	// Zoom control
	if ( is.rising( IK_X ) ) zoomOut();
	if ( is.rising( IK_Y ) ) zoomIn();

	// Movement
	// X and Y buttons used for zoom; no rotation
	InputSet dummy( is );
	dummy.put( IK_X, false );
	dummy.put( IK_Y, false );
	eu->input( dummy );
}

/*
================================
Camera::update

Follows the target (if there is one)
with a velocity proportional to the distance from the target.
This effects an exponential decay in the distance from the target:
the camera moves rapidly when the target is far away
and slows down as it gets close.
================================
*/
void Camera::update()
{
	Entity* target = getTarget();
	if ( target ) {
		Vec2 center = target->getAABB().center();
		Vec2 response( CAMERA_RESPONSE_X, CAMERA_RESPONSE_Y );
		eu->setVelocity( Vec2::prod( center - eu->getPosition(), response ) );
	}

	// Manually integrate Z position
	dz = ( zoom_distance( zoom ) - z ) * CAMERA_RESPONSE_Z;
	z += dz;
}

/*
================================
Camera::getAABB
================================
*/
AABB Camera::getAABB() const
{
	return eu->getAABB();
}

/*
================================
Camera::addTarget
================================
*/
void Camera::addTarget( Entity* en )
{
	targets.push_back( en );
}

/*
================================
Camera::getTarget
================================
*/
Entity* Camera::getTarget() const
{
	if ( targets.empty() ) return 0;
	return targets.front();
}

/*
================================
Camera zooming
================================
*/

void Camera::zoomOut()
{
	zoom++;
	clamp( zoom, 0, 5 );
}

void Camera::zoomIn()
{
	zoom--;
	clamp( zoom, 0, 5 );
}
