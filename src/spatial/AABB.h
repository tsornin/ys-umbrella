#ifndef SPATIAL_AABB_H
#define SPATIAL_AABB_H

#include "Vec2.h"

/*
================================
Represents an axis-aligned bounding box
(a bounding box with edges parallel to the coordinate axes).

The region enclosed by an AABB is considered semiclosed
(the two minimum edges are closed, and the two maximum edges are open).

Used for broad-phase collision regions.
================================
*/
struct AABB
{
public: // Members
	Vec2
		min, // bottom-left
		max; // top-right

public:	// Lifecycle
	AABB();
	AABB( const Vec2& v );
	AABB( const Vec2& min, const Vec2& max );

public:	// Collision
	bool intersects( const AABB& box ) const;
	bool contains( const AABB& box ) const;
	bool contains( const Vec2& v ) const;

public:	// Self
	Vec2 center() const;
	Scalar width() const;
	Scalar height() const;

public:	// Other
	AABB& operator += ( const AABB& box );
	const AABB operator + ( const AABB& box ) const;
	AABB fatter( Scalar x ) const;
	void fatten( Scalar x );
	AABB subdiv( unsigned int quad ) const;
};

#endif
