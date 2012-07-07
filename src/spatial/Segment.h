#ifndef SPATIAL_SEGMENT_H
#define SPATIAL_SEGMENT_H

#include "Vec2.h"

/*
================================
Represents a line segment.
================================
*/
struct Segment
{
public: // Members
	Vec2
		first,
		second;

public: // Lifecycle
	Segment();
	Segment( const Vec2& a, const Vec2& b );

public: // Collision
	Vec2 nearest( const Vec2& p );
	Vec2 nearest( const Vec2& p, const Vec2& n );
	std::pair < bool, Vec2 > intersects( const Segment& seg );
};

#endif
