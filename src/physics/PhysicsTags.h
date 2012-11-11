#ifndef PHYSICS_TAGS_H
#define PHYSICS_TAGS_H

#include "Handler.h"

typedef unsigned int PhysicsMask;

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

public: // PhysicsTags
	static bool pid_lt( PhysicsTags* a, PhysicsTags* b );

public: // Accessors
	PhysicsMask getMask() { return mask; }
	Handler* getOwner() { return owner; }

public: // Mutators
	void setMask( PhysicsMask m ) { mask = m; }
	void setOwner( Handler* h ) { owner = h; }

public: // Members
	int pid;
	PhysicsMask mask;
	bool expire_enable;
	Handler* owner;
};

#endif
