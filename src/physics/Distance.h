#ifndef PHYSICS_DISTANCE_CONSTRAINT_H
#define PHYSICS_DISTANCE_CONSTRAINT_H

#include "PhysicsTags.h"
#include <set> // for std::set < Angular* >
#include "spatial/Scalar.h"

class AABB;
class Verlet;
class Angular;

/*
================================
Enumerates different types of distance constraints:
"pull": only applies when stretched
"push": only applies when squashed
"hard": always applies

Not to be confused with DistanceType (the collision type for Distance).
================================
*/
enum DCType
	{ DC_HARD, DC_PULL, DC_PUSH };

/*
================================
Distance constraint.

Representse a distance constraint between two Verlet particles.

Used to form soft-bodies. For example,
A line of masses-and-constraints forms a string,
a grid of masses-and-constraints forms a cloth, and
a tree of masses-and-constraints forms a wiggly plant.

Instance of this class are managed by the physics engine.
Used PhysicsState::createDistance to create a Distance constraint.
================================
*/
class Distance : public PhysicsTags
{
private: // Lifecycle
	Distance( Verlet* a, Verlet* b );
	friend class PhysicsState;

public: // "Entity" functions
	void apply();
	AABB getAABB() const;

public: // Distance functions

public: // Accessors

public: // Mutators
	void setPower( Scalar k ) { power = k; }
	void setType( DCType t ) { type = t; }

private: // Members
	// Vertices
	Verlet
		*a, // source vertex
		*b; // target vertex

	// Constraint properties
	Scalar rest_length;
	Scalar power; // in range [ 0, 1 ]
	DCType type;

private: // Physics engine graph data
	std::set < Angular* > edges;
};

#endif
