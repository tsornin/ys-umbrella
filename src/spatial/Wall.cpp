#include "Wall.h"

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
