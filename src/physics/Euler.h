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
	friend class PhysicsState;

public: // "Entity" functions
	void input( const InputSet& is );
	void update();
	AABB getAABB() const;
	friend class Renderer;

public: // Euler functions

public: // Accessors
	Scalar getX() const { return position.x; }
	Scalar getY() const { return position.y; }

	Vec2 getPosition() const { return position; }
	Vec2 getVelocity() const { return velocity; }
	bool isLinearEnable() const { return linear_enable; }

	Vec2 getGravity() const { return gravity; }

	Scalar getMass() const { return mass; }
	Scalar getBounce() const { return bounce; }

public: // Mutators (set)
	void setX( Scalar x ) { position.x = x; }
	void setY( Scalar y ) { position.y = y; }

	void setPosition( const Vec2& pos ) { position = pos; }
	void setVelocity( const Vec2& vel ) { if ( linear_enable ) { velocity = vel; } }
	void setLinearEnable( bool le ) { linear_enable = le; }

	void setLinearDamping( Scalar ld ) { linear_damping = ld; }

	void setGravity( Vec2 g ) { gravity = g; }

	void setMass( Scalar m ) { mass = m; }
	void setBounce( Scalar b ) { bounce = b; }

public: // Mutators (add)
	void addPosition( const Vec2& add ) { if ( linear_enable ) { position += add; } }
	void addVelocity( const Vec2& add ) { if ( linear_enable ) { velocity += add; } }

private: // Members
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
};

/*
// New access control
// NOTE: velocity-LE invariant set by update()
{
public:
	position;
	velocity;
	linear_enable;
	gravity;

private:
	linear_damping; // clamp [ 0, 1 ]
	mass; // clamp ( 0, oo )
	bounce; // clamp [ 0, 1 ]

public:
	setLinearDamping();
	setMass();
	setBounce();
}
*/
#endif
