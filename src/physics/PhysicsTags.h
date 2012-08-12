#ifndef PHYSICS_TAGS_H
#define PHYSICS_TAGS_H

#include "Handler.h"

// TODO: this seems silly
typedef unsigned int EulerType;
typedef unsigned int RigidType;
typedef unsigned int VerletType;
typedef unsigned int DistanceType;

/*
================================
Tags used by the physics engine,
common to all physics objects.
================================
*/
struct PhysicsTags
{
public: // Lifecycle
	PhysicsTags();
	friend class PhysicsState;

	bool expired() const;

public: // Accessors
	unsigned int getMask() { return mask; }
	Handler* getOwner() { return owner; }

public: // Mutators
	void setMask( unsigned int m ) { mask = m; }
	void setOwner( Handler* h ) { owner = h; }

public: // Members
	int pid;
	unsigned int mask;
	bool expire_enable;
	Handler* owner;
};

#endif
