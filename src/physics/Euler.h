#ifndef PHYSICS_EULER_H
#define PHYSICS_EULER_H

#include "PhysicsTags.h"
#include "spatial/Vec2.h"

class AABB;
class InputSet;

/*
================================
Euler particle.

Provides a dedicated particle type for unstable particle simulations.

Instances of this class are managed by the physics engine.
Use PhysicsState::createEuler to create an Euler particle.
================================
*/
class Euler : public PhysicsTags
{
private: // Lifecycle
	Euler();
	Euler( const Euler& ) = delete;
	Euler& operator = ( const Euler& ) = delete;
	~Euler() = default;

public: // "Entity" functions
	void input( const InputSet& is );
	void update();
	AABB getAABB() const;

public: // Euler functions
	bool frozen() const { return !linear_enable; }

	Scalar getX() const { return position.x; }
	Scalar getY() const { return position.y; }

	void setPosition( const Vec2& pos ) { position = pos; }
	void setVelocity( const Vec2& vel ) { if ( linear_enable ) { velocity = vel; } }

	void addPosition( const Vec2& add ) { if ( linear_enable ) { position += add; } }
	void addVelocity( const Vec2& add ) { if ( linear_enable ) { velocity += add; } }

public: // Members
	// Position state
	Vec2
		position, // in 2D world-space
		velocity; // in units/frame
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

private:
	std::list < Euler* >::iterator it;

	friend class PhysicsState;
	template < typename T > friend struct Expire;
};

#endif
