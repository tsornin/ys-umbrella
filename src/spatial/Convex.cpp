#include "Convex.h"
#include "AABB.h" // for Convex::getAABB
#include "Wall.h" // for Convex::nearest
#include "Segment.h"

/*
================================
Convex::Convex
================================
*/
Convex::Convex( const std::vector < Vec2 >& points ) :
	points( points )
{
	// Compute normals
	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		const Vec2& p = points[i]; // current point
		const Vec2& q = points[ (i+1) % n ]; // next point
		// Expected CCW: right-hand normal faces outwards
		Vec2 normal = (q - p).rperp();
		normal.normalize();
		normals.push_back( normal );
	}

	// TODO: we shouldn't have to do this
	verify();
}

/*
================================
Convex::negation

Returns a Convex C such that for all points p,
iff this->contains( p ) then C.contains( -p ).
================================
*/
const Convex Convex::negation() const
{
	Convex ret( *this );

	int n = ret.points.size();
	for ( int i = 0; i < n; ++i ) {
		ret.points[i] = -ret.points[i];
	}
	for ( int i = 0; i < n; ++i ) {
		ret.normals[i] = -ret.normals[i];
	}

	return ret;
}

/*
================================
Convex::contains (overloaded)

Returns true if this polygon contains the specified point
(point-in-polygon query).
================================
*/
bool Convex::contains( const Vec2& p ) const
{
	// A convex polygon can be described
	// as an intersection of half-spaces.
	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		Wall w( points[i], normals[i] );
		if ( !w.contains( p ) ) return false;
	}
	return true;
}

/*
================================
Convex::correction (overloaded)

Returns:
	bool	true if this polygon contains the specified point
	Vec2	the (unit) axis along which the minimum distance was found
	Scalar	the minimum distance from the specified point to this Convex

When combined, the second half of the return value gives
the minimum correction to move the specified point out of this polygon.
================================
*/
std::pair < bool, std::pair < Vec2, Scalar > >
Convex::correction( const Vec2& p ) const
{
	std::pair < bool, std::pair < Vec2, Scalar > > ret;
	Scalar overlap = SCALAR_MAX;

	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		// Each normal might be a separating axis
		const Vec2& axis = normals[i];
		// The maximum projection of the polygon corresponds to this normal
		Scalar max_b = points[i].dot( axis );
		// The minimum projection of the point is...the point
		Scalar min_a = p.dot( axis );

		if ( max_b <= min_a ) {
			// Not overlapping. Separation found; early exit.
			ret.first = false;
			return ret;
		}
		else {
			// Overlapping; keep minimum overlap.
			Scalar overlap_c = max_b - min_a;
			if ( overlap_c < overlap ) {
				overlap = overlap_c;
				ret.second.first = axis;
				ret.second.second = overlap_c;
			}
		}
	}

	// Didn't exit; no separation.
	ret.first = true;
	return ret;
}

/*
================================
Convex::correction (overloaded)

Returns:
	bool	true if this polygon contains the specified point
	Vec2	the biased minimum correction to move the specified point
			out of this polygon
================================
*/
std::pair < bool, Vec2 >
Convex::correction( const Vec2& p, const Vec2& bias ) const
{
	std::pair < bool, Vec2 > ret;
	Scalar overlap = SCALAR_MAX;

	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		const Vec2& axis = normals[i];
		Scalar max_b = points[i].dot( axis );
		Scalar min_a = p.dot( axis );

		if ( max_b <= min_a ) {
			ret.first = false;
			return ret;
		}
		else {
			// Keep the biased minimum overlap.
			Scalar overlap_c = max_b - min_a;
			Scalar overlap_c_biased = overlap_c + bias.dot( axis );
			if ( overlap_c_biased < overlap ) {
				overlap = overlap_c;
				ret.second = axis * overlap_c;
			}
		}
	}

	ret.first = true;
	return ret;
}

/*
================================
Convex::nearest

Returns the point on this polygon nearest to the specified point.
================================
*/
Vec2 Convex::nearest( const Vec2& p ) const
{
	Vec2 ret;
	Scalar score = SCALAR_MAX;

	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		// View each edge as a segment
		Segment s( points[ i ], points[ (i+1) % n ] );

		// Pick the closest result
		Vec2 ret_c = s.nearest( p, normals[i].lperp() );
		Scalar score_c = (p - ret_c).length2();
		if ( score_c < score ) {
			score = score_c;
			ret = ret_c;
		}
	}

	return ret;
}

/*
================================
Convex::getAABB

Computes the AABB of this polygon.
================================
*/
AABB Convex::getAABB() const
{
	AABB ret( points[0] );

	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		ret += points[i];
	}

	return ret;
}

/*
================================
Convex::verify

Returns true if this polygon is convex.
If the polygon is convex but specified clockwise,
the points and normals are re-ordered to be counter-clockwise.

Empty polygons are rejected (no shape).
One-point polygons are rejected (no normal).
Two-point polygons are always accepted (digons are fine).
================================
*/
bool Convex::verify()
{
	int n = points.size();

	// 0- and 1-point degenerate shapes rejected.
	if ( n < 2 ) return false;
	// Digons (lines) allowed.
	if ( n < 3 ) return true;

	// This should end up equal to 2*PI
	Scalar angle_turned = 0.0;
	bool ccw = true;
	bool cw = true;

	// If the points of a polygon are specified counter-clockwise,
	// we should always be "turning" left as we traverse the edges
	// of the polygon. In other words, the cross product of any edge
	// and the next edge should always be positive.
	for ( int i = 0; i < n; ++i ) {
		// Examine each edge.
		Vec2& p0 = points[i];
		// The next two points (loop around)
		Vec2& p1 = points[ (i+1) % n ];
		Vec2& p2 = points[ (i+2) % n ];

		// Edge vectors
		Vec2 v01( p0, p1 );
		Vec2 v12( p1, p2 );

		// Convex polygons will have (cw ^ ccw) when we're done.
		if ( v01.cross( v12 ) < 0 )
			ccw = false;
		else
			cw = false;

		angle_turned += Vec2::angle( v01, v12 );
	}

	// NOTE: This shouldn't happen...
	if ( cw == ccw ) return false;

	// This traversal checks for CCW vertices, but not for "spirals".
	// For "spirals", we keep track of the total angle turned, which
	// should end up as approximately (numerical error) TWO_PI.
	if ( !equals( angle_turned, 2*PI ) ) return false;
	
	// One of these two is guaranteed true
	// since we checked (cw==ccw) earlier.
	
	// Counter-clockwise? We're good.
	if ( ccw ) return true;
	
	// Clockwise? No problem!
	// If we notice that the polygon is specified clockwise (i.e.
	// the turned-angle is -TWO_PI), we can fix it here (cool!).
	if ( cw ) {
		// Reverse order of points
		int a, b;
		Vec2 tmp;
		for ( a = 0, b = n-1; a < n/2; a++, b-- ) {
			tmp = points[a];
			points[a] = points[b];
			points[b] = tmp;
		}
		// Don't forget to recalculate normals
		normals.clear();
		for ( int i = 0; i < n; ++i ) {
			Vec2& p = points[i]; // current point
			Vec2& q = points[ (i+1) % n ]; // next point
			// Expected CCW; right-hand normal faces outwards
			Vec2 normal = ( q - p ).rperp();
			normal.normalize();
			normals.push_back( normal );
		}
		return true;
	}

	// Never happens.
	return true;
}

/*
================================
Convex::calculate

Calculates the area, centroid, and moment (about the centroid) of this polygon.
================================
*/
void Convex::calculate( Scalar& area, Scalar& moment, Vec2& centroid ) const
{
	int n = points.size();
	switch ( n )
	{
	case 0: { // No points, no area!
		area = 0;
		moment = 0;
		centroid = Vec2( 0 );
	}
	break;

	case 1: { // Treat single dots as point-masses
		area = 1.0;
		moment = 1.0;
		centroid = points[0];
	}
	break;

	case 2: { // Treat two dots as a thin rod
		area = 0;
		moment = 0;
		centroid = Vec2( 0 );
		const Vec2& p = points[0];
		const Vec2& q = points[1]; // guaranteed two points
		Scalar length = ( q - p ).length(); // edge length
		area += length * 1.0; // this isn't -really- correct
		moment += area * length * length / 12.0; // rod MoI formula
		centroid = ( p + q ) * 0.5; // edge center
	}
	break;

	default: { // Treat everything else as a general convex polygon
		area = 0;
		moment = 0;
		centroid = Vec2( 0 );
		for ( int i = 0; i < n; ++i ) {
			const Vec2& p0 = points[i]; // current point
			const Vec2& p1 = points[ (i+1) % n ]; // next point
			// The value of the cross product is the area of a parallelogram.
			Scalar cross = p0.cross( p1 ); // should be positive (CCW)
			// Area summation.
			area += cross;
			// Centroid summation.
			centroid.x += ( p0.x + p1.x ) * cross;
			centroid.y += ( p0.y + p1.y ) * cross;
			// Moment summation.
			moment += ( p0*p0 + p0*p1 + p1*p1 ) * cross;
		}

		// TODO: assert that moment/area are positive

		// Normalize area sum.
		area /= 2.0;
		// Normalize moment sum.
		moment /= 12.0;
		// Normalize centroid sum (using the area).
		centroid /= ( 6.0 * area );

		// We actually want the moment around the centroid, not the origin.
		Vec2 r = centroid;
		moment -= area * ( r*r ); // (parallel axis theorem)
	}
	break;
	}
}

/*
================================
Convex::translate
================================
*/
void Convex::translate( Vec2 p )
{
	int n = points.size();
	for ( int i = 0; i < n; ++i ) {
		points[i] += p;
	}
}

/*
================================
Convex::rotate
================================
*/
void Convex::rotate( Scalar rad )
{
	Scalar cos = std::cos( rad );
	Scalar sin = std::sin( rad );

	int n = points.size();

	for ( int i = 0; i < n; ++i ) {
		points[i] = points[i].rotation( cos, sin );
	}

	for ( int i = 0; i < n; ++i ) {
		normals[i] = normals[i].rotation( cos, sin );
	}
}

/*
================================
Convex::transform
================================
*/
void Convex::transform( Vec2 p, Scalar t )
{
	rotate( t );
	translate( p );
}
