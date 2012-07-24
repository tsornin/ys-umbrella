#ifndef PHYSICS_EULER_H
#define PHSYICS_EULER_H

#include "PhysicsTags.h"
#include "spatial/Vec2.h"

class AABB;
class InputSet;

/*
================================
Euler particle.

Provides a dedicated particle type for unstable particle simulations.

Instances of this class are managed by the physics engine.
Used PhysicsState::createEuler to create an Euler particle.
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

public: // Euler functions

public: // Accessors
	Scalar getX() const { return position.x; }
	Scalar getY() const { return position.y; }

	Vec2 getPosition() const { return position; }
	Vec2 getVelocity() const { return velocity; }
	bool isLinearEnable() const { return linear_enable; }

	Scalar getAngle() const { return angle; }
	Scalar getAngularVelocity() const { return angular_velocity; }
	bool isAngularEnable() const { return angular_enable; }

	Scalar getMass() const { return mass; }
	Scalar getMoment() const { return moment; }
	Scalar getBounce() const { return bounce; }

public: // Mutators (set)
	void setX( Scalar x ) { position.x = x; }
	void setY( Scalar y ) { position.y = y; }

	void setPosition( const Vec2& pos ) { position = pos; }
	void setVelocity( const Vec2& vel ) { if ( linear_enable ) { velocity = vel; } }
	void setLinearEnable( bool le ) { linear_enable = le; }

	void setAngle( const Scalar th ) { angle = th; }
	void setAngularVelocity( const Scalar w ) { if ( angular_enable ) { angular_velocity = w; } }
	void setAngularEnable( bool ae ) { angular_enable = ae; }

	void setLinearDamping( Scalar ld ) { linear_damping = ld; }
	void setAngularDamping( Scalar ad ) { angular_damping = ad; }

	void setGravity( Vec2 g ) { gravity = g; }

	void setMass( Scalar m ) { moment *= (mass/m); mass = m; }
	void setBounce( Scalar b ) { bounce = b; }

public: // Mutators (add)
	void addPosition( const Vec2& add ) { if ( linear_enable ) { position += add; } }
	void addVelocity( const Vec2& add ) { if ( linear_enable ) { velocity += add; } }

	void addAngle( const Scalar th ) { if ( angular_enable ) { angle += th; } }
	void addAngularVelocity( const Scalar w ) { if ( angular_enable ) { angular_velocity += w; } }

private: // Members
	// Position state
	Vec2
		position, // in 2D world-space
		velocity, // in units/frame
		previous;
	bool linear_enable; // Must enable this to move.

	// Rotation state
	Scalar
		angle, // in radians
		angular_velocity; // in radians/frame
	bool angular_enable; // Must enable this to rotate.

	// Damping
	Scalar
		linear_damping,
		angular_damping;

	// Gravity
	Vec2
		gravity;

	// Collision properties
	Scalar
		mass, // area in 2D
		moment, // moment of inertia
		bounce; // restitution
};

#endif
