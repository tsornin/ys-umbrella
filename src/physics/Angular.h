#ifndef PHYSICS_ANGULAR_CONSTRAINT_H
#define PHYSICS_ANGULAR_CONSTRAINT_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"

class Verlet;
class Distance;

/*
================================
"Angular constraint"; more accurately, a box clamp.

Represents an angular constraint between two distance constraints.

Instances of this class are managed by the physics engine.
Use PhysicsState::createAngular to create an Angular constraint.

TODO: under construction.
================================
*/
class Angular :
	public PhysicsTags,
	public PhysicsGraph < Distance, Angular >::Edge
{
public: // Lifecycle
	Angular( Distance* m, Distance* n );
	Angular( const Angular& ) = delete;
	Angular& operator = ( const Angular& ) = delete;
	~Angular() = default;

private: // Members
	Verlet *vla, *vlb, *vlc;

	// ???
	Verlet *vll, *vlr;
	Distance
		*al, *ar,
		*bl, *br,
		*cl, *cr,
		*h, *v;

	friend class PhysicsState;
	friend class Renderer;
};

#endif
