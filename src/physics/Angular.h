#ifndef PHYSICS_ANGULAR_CONSTRAINT_H
#define PHYSICS_ANGULAR_CONSTRAINT_H

#include "PhysicsTags.h"
class Verlet;
class Distance;

/*
================================
"Angular constraint"; more accurately, a box clamp.

Represents an angular constraint between two distance constraints.

Instances of this class are managed by the physics engine.
Use PhysicsState::createAngular to create an Angular constraint.
================================
*/
class Angular : public PhysicsTags
{
public: // Lifecycle
	Angular( Distance* m, Distance* n );
	friend class PhysicsState;

private: // Members
	// Vertices
	Distance
		*m, // source vertex
		*n; // target vertex

	Verlet *a, *b, *c;

	// ???
	Verlet *l, *r;
	Distance
		*al, *ar,
		*bl, *br,
		*cl, *cr,
		*h, *v;
};

#endif
