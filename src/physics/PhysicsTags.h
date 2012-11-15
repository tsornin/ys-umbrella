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

public: // PhysicsTags
	bool expired() const;
	static bool pid_lt( PhysicsTags* a, PhysicsTags* b );

public: // Members
	PhysicsMask mask;
	Handler* owner;

private: // Members
	int pid;
	bool expire_enable;

	friend class PhysicsState;
};

#endif
