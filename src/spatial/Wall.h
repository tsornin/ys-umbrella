#ifndef SPATIAL_WALL_H
#define SPATIAL_WALL_H

#include "Vec2.h"

/*
================================
Represents an open half-space (a plane).
================================
*/
struct Wall
{
public: // Members
	Vec2
		origin, // a point on the wall
		normal; // points outward

public: // Lifecycle
	Wall();
	Wall( const Vec2& o, const Vec2& n );

public: // Collision
	Scalar distance( const Vec2& p ) const;
	bool contains( const Vec2& p ) const;
	Vec2 nearest( const Vec2& p ) const;
};

#endif
