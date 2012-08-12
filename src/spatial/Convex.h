#ifndef SPATIAL_CONVEX_H
#define SPATIAL_CONVEX_H

#include <vector> // for std::vector< Vec2 >
#include "Vec2.h" // for std::vector< Vec2 >

class AABB;

/*
================================
Represents a convex polygon with at least two points.

Convex objects with zero or one points are errors
(since they have no meaningful normals).

Non-convex shapes are errors
(since separating-axis collision won't work on them).

Points are specified counter-clockwise.
================================
*/
struct Convex
{
public: // Members
	std::vector < Vec2 > points;
	std::vector < Vec2 > normals;

public: // Lifecycle
	Convex( const std::vector < Vec2 >& points );

public: // Convex
	const Convex negation() const;
	bool contains( const Vec2& p ) const;
	Vec2 nearest( const Vec2& p ) const;
	AABB getAABB() const;

public: // Other
	std::pair < bool, std::pair < Vec2, Scalar > > correction( const Vec2& p ) const;
	// Return correction, normal, and boolean.
	std::pair < bool, Vec2 > correction( const Vec2& p, const Vec2& bias ) const;

public: // Self
	bool verify();
	void calculate( Scalar& area, Scalar& moment, Vec2& centroid ) const;

public: // Movement
	void translate( Vec2 p );
	void rotate( Scalar rad );
	void transform( Vec2 p, Scalar t );
};

#endif
