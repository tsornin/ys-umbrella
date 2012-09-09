#include "Wall.h"
#include "Convex.h"

/*
================================
Wall::Wall (overloaded)
================================
*/
Wall::Wall()
{
	
}

/*
================================
Wall::Wall (overloaded)

n	a unit vector
================================
*/
Wall::Wall( const Vec2& o, const Vec2& n ) :
	origin(o), normal(n)
{
	
}

/*
================================
Wall::distance

Returns the distance from this plane to the specified point
(a negative value if this plane contains the specified point).
================================
*/
Scalar Wall::distance( const Vec2& p ) const
{
	return (p - origin).dot( normal );
}

/*
================================
Wall::contains

Returns true if this plane contains the specified point.
================================
*/
bool Wall::contains( const Vec2& p ) const
{
	return distance( p ) < 0;
}

/*
================================
Wall::nearest

Returns the point on this plane nearest to the specified point.
================================
*/
Vec2 Wall::nearest( const Vec2& p ) const
{
	return p - normal * distance( p );
}

/*
================================
Wall::shadow
================================
*/
Scalar Wall::shadow( const Vec2& p ) const
{
	return (p - origin).dot( normal.rperp() );
}

/*
================================
Wall::distance
================================
*/
Scalar Wall::distance( const Convex& c ) const
{
	Scalar min = SCALAR_MAX;

	for ( const Vec2& p : c.points ) {
		min = std::min( min, distance( p ) );
	}

	return min;
}

/*
================================
Wall::contains
================================
*/
bool Wall::contains( const Convex& c ) const
{
	return distance( c ) < 0;
}

/*
================================
Wall::shadow
================================
*/
std::pair < Scalar, Scalar > Wall::shadow( const Convex& c ) const
{
	Scalar min = SCALAR_MAX;
	Scalar max = -SCALAR_MAX;

	for ( const Vec2& p : c.points ) {
		Scalar s = shadow( p );
		min = std::min( min, s );
		max = std::max( max, s );
	}

	return std::pair < Scalar, Scalar >( min, max );
}
