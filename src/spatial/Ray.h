#ifndef SPATIAL_RAY_H
#define SPATIAL_RAY_H

#include "Vec2.h"

class AABB;
class Segment;

/*
================================
Represents a ray.
================================
*/
struct Ray
{
public: // Members
	Vec2
		origin,
		direction;

public: // Lifecycle
	Ray( const Vec2& o, const Vec2& d );

public: // Collision
	bool intersects( const AABB& box ) const;
	std::pair < bool, Scalar > intersects( const Ray& ray ) const;
	std::pair < bool, Scalar > intersects( const Segment& seg ) const;

public: // Self
	Vec2 at( Scalar t ) const;
};

#endif
