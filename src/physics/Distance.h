#ifndef PHYSICS_DISTANCE_CONSTRAINT_H
#define PHYSICS_DISTANCE_CONSTRAINT_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include "spatial/Vec2.h"

class AABB;
class Verlet;
class Angular;

/*
================================
Enumerates different types of distance constraints:
"pull": only applies when stretched
"push": only applies when squashed
"hard": always applies
================================
*/
enum DistanceType
	{ DC_HARD, DC_PULL, DC_PUSH };

/*
================================
Distance constraint.

Represents a distance constraint between two Verlet particles.

Used to form soft-bodies. For example,
A line of masses-and-constraints forms a string,
a grid of masses-and-constraints forms a cloth, and
a tree of masses-and-constraints forms a wiggly plant.

Instance of this class are managed by the physics engine.
Use PhysicsState::createDistance to create a Distance constraint.
================================
*/
class Distance :
	public PhysicsTags,
	public PhysicsGraph < Verlet, Distance >::Edge,
	public PhysicsGraph < Distance, Angular >::Vertex
{
private: // Lifecycle
	Distance( Verlet* a, Verlet* b );
	Distance( const Distance& ) = delete;
	Distance& operator = ( const Distance& ) = delete;
	~Distance() = default;

public: // "Entity" functions
	AABB getAABB() const;

public: // Distance functions
	void apply();

public: // Members
	// Constraint properties
	Scalar power; // in range [ 0, 1 ]
	DistanceType type;

private: // Members
	Scalar rest_length;

	friend class PhysicsState;
	template < typename T > friend struct Expire;
};

#endif
