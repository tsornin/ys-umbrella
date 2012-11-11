#ifndef PHYSICS_VERLET_H
#define PHYSICS_VERLET_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include <set> // for std::set < Distance* >
#include "spatial/Vec2.h"

class AABB;
class Distance;

/*
================================
Verlet particle.

Provides a dedicated particle type for stable particle simulations.

Instances of this class are managed by the physics engine.
Use PhysicsState::createVerlet to create a Verlet particle.

NOTE: Use Verlet::putPosition, not Verlet::setPosition,
to specify a starting position during initialization
(setPosition implicitly modifies velocity).
================================
*/
class Verlet :
	public PhysicsTags,
	public PhysicsGraph < Verlet, Distance >::Vertex
{
private: // Lifecycle
	Verlet();
	friend class PhysicsState;

public: // "Entity" functions
	void update();
	AABB getAABB() const;
	friend class Renderer;

public: // Verlet functions
	void putPosition( const Vec2& pos );

public: // Accessors
	bool frozen() const { return !linear_enable; }

	Vec2 getPosition() const { return position; }
	Vec2 getVelocity() const { return velocity; }
	bool isLinearEnable() const { return linear_enable; }

	Vec2 getGravity() const { return gravity; }

	Scalar getMass() const { return mass; }
	Scalar getBounce() const { return bounce; }

public: // Mutators (set)
	void setPosition( const Vec2& pos ) { if ( linear_enable ) { position = pos; } }
	void setLinearEnable( bool le ) { linear_enable = le; }

	void setGravity( const Vec2& g ) { gravity = g; }

	void setMass( Scalar m ) { mass = m; }
	void setBounce( Scalar b ) { bounce = b; }

public: // Mutators (add)
	void addPosition( const Vec2& add ) { if ( linear_enable ) { position += add; } }

private: // Members
	// Position state
	Vec2
		position, // in 2D world-space
		velocity, // in units/frame
		previous; // implicit velocity
	bool linear_enable; // Must enable this to move.

	// Damping
	Scalar
		linear_damping;

	// Gravity
	Vec2
		gravity;

	// Collision properties
	Scalar
		mass,
		bounce;
};

#endif
