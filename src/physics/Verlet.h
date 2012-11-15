#ifndef PHYSICS_VERLET_H
#define PHYSICS_VERLET_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include "spatial/Vec2.h"

class AABB;
// class InputSet;
class Distance;

/*
================================
Verlet particle.

Provides a dedicated particle type for stable particle simulations.

Instances of this class are managed by the physics engine.
Use PhysicsState::createVerlet to create a Verlet particle.

NOTE: Use Verlet::putPosition to specify a starting position
during initialization (setPosition implicitly modifies velocity).
================================
*/
class Verlet :
	public PhysicsTags,
	public PhysicsGraph < Verlet, Distance >::Vertex
{
private: // Lifecycle
	Verlet();
	Verlet( const Verlet& ) = delete;
	Verlet& operator = ( const Verlet& ) = delete;
	~Verlet() = default;

public: // "Entity" functions
	// void input( const InputSet& is );
	void update();
	AABB getAABB() const;

public: // Verlet functions
	bool frozen() const { return !linear_enable; }

	void setPosition( const Vec2& pos ) { if ( linear_enable ) { position = pos; } }
	void addPosition( const Vec2& add ) { if ( linear_enable ) { position += add; } }

	void putPosition( const Vec2& pos );

public: // Members
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

	friend class PhysicsState;
	template < typename T > friend struct Expire;
};

#endif
