#include "Segment.h"
#include "Wall.h" // for Segment::nearest

/*
================================
Segment::Segment (overloaded)
================================
*/
Segment::Segment()
{
	
}

/*
================================
Segment::Segment (overloaded)
================================
*/
Segment::Segment( const Vec2& a, const Vec2& b ) :
	first(a), second(b)
{
	
}

/*
================================
Segment::nearest

Returns the point on this segment nearest to the specified point.
================================
*/
Vec2 Segment::nearest( const Vec2& p )
{
	return nearest( p, (second - first).unit() );
}

/*
================================
Segment::nearest

Returns the point on this segment nearest to the specified point.

n	the precomputed unit vector (second - first).unit()
================================
*/
Vec2 Segment::nearest( const Vec2& p, const Vec2& n )
{
	// Points behind an endpoint map to that endpoint
	Wall w_a( first, n );
	if ( w_a.contains( p ) ) return first;

	Wall w_b( second, -n );
	if ( w_b.contains( p ) ) return second;

	// Points between endpoints map onto the segment
	Wall w_s( first, n.lperp() );
	return w_s.nearest( p );
}

/*
================================
Segment::intersects

Returns:
	bool	true if this segment intersects the specified segment
	Vec2	the point where the segments intersect

http://paulbourke.net/geometry/lineline2d/

We can describe both segments parametrically:
	S(t) = first + (second - first) * t,
	t in [ 0, 1 ]

Solve for intersection:
	A(t_a) = B(t_b)

Verify bounds:
	t_a in [ 0, 1 ]
	t_b in [ 0, 1 ]
================================
*/
std::pair < bool, Vec2 >
Segment::intersects( const Segment& seg )
{
	std::pair < bool, Vec2 > ret;

	Vec2 a = second - first;
	Vec2 b = seg.second - seg.first;
	Vec2 z = first - seg.first;

	// TODO: maybe not very robust (parallel segments?)
	Scalar t_a = (b ^ z) / (a ^ b);
	Scalar t_b = (a ^ z) / (a ^ b);

	ret.first = (
			((t_a > 0) && (t_a < 1)) &&
			((t_b > 0) && (t_b < 1)) );
	ret.second = first + a * t_a;
	// ret.second = seg.first + b * t_b;
	return ret;
}
