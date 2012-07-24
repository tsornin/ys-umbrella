#ifndef ENTITY_H
#define ENTITY_H

#include "EntityState.h" // for ES reference
#include "game/InputSet.h" // for Entity::input
#include "spatial/AABB.h" // for Entity::getAABB
#include "physics/Handler.h"

typedef int Renderer; // TODO: renderer

/*
================================
The Entity interface represents arbitrary "scripted" behaviors.
Entities are expected to contain (composition) physics objects,
but not inherit from them, since an Entity could be composed of
multiple physics objects (or none at all).

Since we need PhysicsState support for deletion of physics objects,
we keep a reference to the PhysicsState we belong to.

Although the functions of physics objects provided by PhysicsState
look similar to Entity functions, physics objects are not Entities
and are exclusive to the physics engine.

Entities can define their location in world-space (Entity::getAABB).
This is generally an AABB containing all physics objects of this Entity.

Entities can be composed of sub-entities.
When this happens, Entity::update is run recursively.
================================
*/
class Entity : public Handler
{
public: // Life cycle
	Entity( EntityState& es ) : es( es ) {}
	virtual ~Entity() {}

public: // Entity functions
	virtual void input( const InputSet& is ) {}
	virtual void update() {}
	virtual void draw( Renderer& rd ) const {}
	//friend class Renderer;
	virtual AABB getAABB() const { return AABB( Vec2(0) ); }

public: // Handler
	virtual void handle() {}

protected: // Members
	friend class EntityState;
	EntityState& es;
	int eid;
};

#endif
