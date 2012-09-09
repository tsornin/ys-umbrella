#ifndef SPATIAL_WALL_H
#define SPATIAL_WALL_H

#include "Vec2.h"

// Shadow
struct Convex;

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

public: // Point
	Scalar distance( const Vec2& p ) const;
	bool contains( const Vec2& p ) const;
	Vec2 nearest( const Vec2& p ) const;
	Scalar shadow( const Vec2& p ) const;

public: // Convex
	Scalar distance( const Convex& c ) const;
	bool contains( const Convex& c ) const;
	std::pair < Scalar, Scalar > shadow( const Convex& c ) const;
};

#endif
