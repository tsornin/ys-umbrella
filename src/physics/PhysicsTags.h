#ifndef PHYSICS_TAGS_H
#define PHYSICS_TAGS_H

// TODO: get rid of this.
typedef int Handler;

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
	Handler* getOwner() { return owner; }

public: // Mutators
	void setOwner( Handler* h ) { owner = h; }

public: // Members
	bool expire_enable;
	int pid;
	Handler* owner;
};

#endif
