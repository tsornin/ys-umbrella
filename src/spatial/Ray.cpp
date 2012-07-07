#include "Ray.h"
#include <algorithm> // for std::min, std::max, std::swap
#include "AABB.h"
#include "Segment.h"

/*
================================
Ray::Ray
================================
*/
Ray::Ray( const Vec2& o, const Vec2& d ) :
	origin(o), direction(d)
{
	
}

/*
================================
Ray::intersects (overloaded)

Returns true if this ray intersects the specified AABB.

This is the "slabs" method developed by Kay and Kayjia:
http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm

A slab is the space between two axis-aligned hyperplanes;
an AABB is the intersection of [num_dimensions] slabs.
We find t_near and t_far for each slab; the ray intersects the box iff
the overall largest t_near is greater than the overall smallest t_far.

TODO: not robust for parallel case
================================
*/
bool Ray::intersects( const AABB& box ) const
{
	if ( box.contains( origin ) )
		return true;

	if ( direction.x == 0 || direction.y == 0 )
		return false;

	Scalar t_near = -SCALAR_MAX;
	Scalar t_far = SCALAR_MAX;

	for ( int i = 0; i < 2; ++i ) {
		Scalar t1 = ( box.min[i] - origin[i] ) / direction[i];
		Scalar t2 = ( box.max[i] - origin[i] ) / direction[i];
		if ( t1 < t2 ) std::swap( t1, t2 );
		t_near = std::max( t_near, t1 );
		t_far = std::max( t_far, t2 );
	}

	return ( t_near > t_far );
}

/*
================================
Ray::intersects (overloaded)

Returns:
	bool	true if this ray intersects the specified ray
	Scalar	the point of intersection as a ray parameter

Unless they are parallel, the lines on which the two rays lie always intersect.
However, the bounds of the ray and segment constrain the intersection.

The description of the ray is parametric:
	R(t) = o + d * t,
	t in [ 0, infinity ).

The description of the other ray is also parametric:
	S(s) = p + e * s,
	s in [ 0, infinity ).

Solve R(t) = S(s):
	o + d * t = p + e * s
	t = (e ^ (p-o)) / (e ^ d)
	s = (d ^ (o-p)) / (d ^ e)

The rays intersect iff t and s are within their respective bounds.

TODO: not robust for parallel case
================================
*/
std::pair < bool, Scalar >
Ray::intersects( const Ray& ray ) const
{
	std::pair < bool, Scalar > ret;

	const Vec2& o = this->origin;
	const Vec2& d = this->direction;
	const Vec2& p = ray.origin;
	const Vec2& e = ray.direction;

	Scalar t = (e ^ (p-o)) / (e ^ d);
	Scalar s = (d ^ (o-p)) / (d ^ e);

	ret.first = (t >= 0) && ( s >= 0 );
	ret.second = t;
	return ret;
}

/*
================================
Ray::intersects (overloaded)

Returns:
	bool	true if this ray intersects the specified segment
	Scalar	the point of intersection as a ray parameter

We can describe the segment parametrically:
	S(s) = a + (b - a) * s,
	s in [ 0, 1 ].

Again, we solve R(t) = S(s).
================================
*/
std::pair < bool, Scalar >
Ray::intersects( const Segment& seg ) const
{
	std::pair < bool, Scalar > ret;

	const Vec2& a = seg.first;
	const Vec2& b = seg.second;
	const Vec2& o = origin;
	const Vec2& d = direction;

	// The numbers blow up if d is parallel to b-a.
	// Use a "straddle check" to cull cases where b and a
	// are on the same side of the ray.
	Vec2 oa( o, a );
	Vec2 ob( o, b );
	if ( oa.rejection( d ) * ob.rejection( d ) >= 0 ) {
		ret.first = false;
		return ret;
	}

	// This is mostly the same math as ray-ray intersection.
	// The difference is that the segment's direction vector, (b-a),
	// isn't normalized.
	// The equation for t stays the same,
	// since e is replaced by ab in both numerator and denominator.
	// The equation for s is changed by a factor of the length of ab,
	// since the allowed range for s is ( 0, 1 )
	// (had ab been normalized, the allowed range for s would be
	// ( 0, ab.length() )).
	Vec2 ab( a, b );
	Vec2 ao( a, o );
	Scalar t = ( ab ^ ao ) / ( d ^ ab );
	// TODO: I think the straddle check makes 's' unnecessary.
	// Scalar s = ( d ^ ao ) / ( d ^ ab );
	// ret.first = (t >= 0) && (s >= 0) && (s <= 1);
	ret.first = t >= 0;
	ret.second = t;
	return ret;
}

/*
================================
Ray::at
================================
*/
Vec2 Ray::at( Scalar t ) const
{
	return origin + direction * t;
}
