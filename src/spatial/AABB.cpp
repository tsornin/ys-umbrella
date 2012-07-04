#include "AABB.h"
#include <algorithm> // for std::min, std::max (in AABB::operator +=)

/*
================================
AABB::AABB (overloaded)
================================
*/
AABB::AABB() :
	min(0), max(0)
{
	
}

/*
================================
AABB::AABB (overloaded)
================================
*/
AABB::AABB( const Vec2& v ) :
	min(v), max(v)
{
	
}

/*
================================
AABB::AABB (overloaded)
================================
*/
AABB::AABB( const Vec2& min, const Vec2& max ) :
	min(min), max(max)
{
	
}

/*
================================
aabb_intersects_sat

Separating-axis style AABB intersection test.

This would be easier if AABB was represented as
{ Vec2 center, Scalar width, Scalar height }.
================================
*/
static bool aabb_intersects_sat( const AABB& a, const AABB& b )
{
	Scalar slop = 1.0;

	// Get the sum of the box dimensions.
	Scalar x_sum = a.width() + b.width() + slop;
	Scalar y_sum = a.height() + b.height() + slop;

	// Get the center difference, along each axis.
	Vec2 diff_center = a.center() - b.center();
	Scalar x_diff = std::fabs( diff_center.x );
	Scalar y_diff = std::fabs( diff_center.y );

	// Check along both axes for separation.
	if ( (x_diff * 2.0) > x_sum ) return false;
	if ( (y_diff * 2.0) > y_sum ) return false;

	// No separation.
	return true;
}

/*
================================
aabb_intersects_minkowski_diff

Minkowski difference style AABB intersection test.

Fast, since the AABB is stored as a min and max position.
================================
*/
static bool aabb_intersects_minkowski( const AABB& a, const AABB& b )
{
	Vec2 slop( 1.0 );
	AABB diff(
		a.min - b.max - slop,
		a.max - b.min + slop );
	return diff.contains( Vec2( 0 ) );
}

/*
================================
AABB::intersects

Returns true if this AABB intersects the specified AABB.
================================
*/
bool AABB::intersects( const AABB& box ) const
{
	// return aabb_intersects_sat( *this, box );
	return aabb_intersects_minkowski( *this, box );
}

/*
================================
AABB::contains

Returns true if this AABB (entirely) contains the specified AABB.
================================
*/
bool AABB::contains( const AABB& box ) const
{
	// Using <= and < satisfies the semiclosed property.
	return (min.x <= box.min.x) && (max.x > box.max.x)
		&& (min.y <= box.min.y) && (max.y > box.max.y);
}

/*
================================
AABB::contains

Returns true if this AABB contains the specified point in world space.
================================
*/
bool AABB::contains( const Vec2& v ) const
{
	// Using <= and < satisfies the semiclosed property.
	return (min.x <= v.x) && (v.x < max.x)
		&& (min.y <= v.y) && (v.y < max.y);
}

/*
================================
"Self" AABB functions
================================
*/

Vec2 AABB::center() const
{
	return (max + min) * 0.5;
}

Scalar AABB::width() const
{
	return ( max.x - min.x );
}

Scalar AABB::height() const
{
	return ( max.y - min.y );
}

/*
================================
AABB::operator +=

Expands this AABB to be the smallest box that contains
both its old size and the specified AABB.

(This isn't a Minkowski sum.)
================================
*/
AABB& AABB::operator += ( const AABB& box )
{
	min.x = std::min( min.x, box.min.x );
	min.y = std::min( min.y, box.min.y );
	max.x = std::max( max.x, box.max.x );
	max.y = std::max( max.y, box.max.y );

	return *this;
}

/*
================================
AABB::operator +

Returns the smallest AABB that contains
both this AABB and the specified AABB.
================================
*/
const AABB AABB::operator + ( const AABB& box ) const
{
	return AABB( *this ) += box;
}

/*
================================
AABB::fatter

Returns a fatter AABB.
================================
*/
AABB AABB::fatter( Scalar x ) const
{
	AABB ret( *this );
	ret.fatten( x );
	return ret;
}

/*
================================
AABB::fatten

Fattens this AABB.
================================
*/
void AABB::fatten( Scalar x )
{
	Vec2 fat( x );
	min -= fat;
	max += fat;
}

/*
================================
AABB::subdiv

Returns an AABB covering the specified quadrant of this AABB.
================================
*/
AABB AABB::subdiv( unsigned int quad ) const
{
	Vec2 q = Vec2::unquadrant( quad );
	Vec2 bounds( width(), height() );
	bounds *= 0.25;
	Vec2 cen = center() + Vec2::prod( q, bounds );
	return AABB( cen - bounds, cen + bounds );
}
