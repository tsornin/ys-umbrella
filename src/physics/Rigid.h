#ifndef PHYSICS_RIGID_H
#define PHYSICS_RIGID_H

#include "PhysicsTags.h"
#include <set>
#include "spatial/Vec2.h"
#include "spatial/Vec3.h"
#include "spatial/Convex.h"

class AABB;
class InputSet;
class Constraint;

/*
================================
Rigid body.

Represents a rigid body with associated collision shapes.

Instances of this class are managed by the physics engine.
Use PhysicsState::createRigid to create a Rigid body.

Rigid bodies with no shapes are errors
(should have used an Euler particle instead).
================================
*/
class Rigid : public PhysicsTags
{
private: // Lifecycle
	Rigid();
	Rigid( const std::vector < Convex >& pgs );
	friend class PhysicsState;

public: // "Entity" functions
	void input( const InputSet& is );
	void update();
	AABB getAABB() const;
	friend class Renderer;

public: // Rigid functions

public: // Accessors
	Scalar getX() const { return position.x; }
	Scalar getY() const { return position.y; }

	Vec3 getPositionState() const { return Vec3( position, angle ); }
	Vec3 getVelocityState() const { return Vec3( velocity, angular_velocity ); }

	Vec3 getInverseMass() const {
		return Vec3(
			Vec2( linear_enable ? 1.0 / mass : 0 ),
			angular_enable ? 1.0 / moment : 0 );
	}

	bool frozen() const { return !linear_enable && !angular_enable; }

	Vec2 getPosition() const { return position; }
	Vec2 getVelocity() const { return velocity; }
	bool isLinearEnable() const { return linear_enable; }

	Scalar getAngle() const { return angle; }
	Scalar getAngularVelocity() const { return angular_velocity; }
	bool isAngularEnable() const { return angular_enable; }

	Vec2 getGravity() const { return gravity; }

	Scalar getMass() const { return mass; }
	Scalar getMoment() const { return moment; }
	Scalar getBounce() const { return bounce; }
	Scalar getFriction() const { return friction; }

public: // Mutators (set)
	void setX( Scalar x ) { position.x = x; }
	void setY( Scalar y ) { position.y = y; }

	void setPositionState( const Vec3& p ) { position = Vec2( p.x, p.y ); angle = p.z; }
	void setVelocityState( const Vec3& v ) { velocity = Vec2( v.x, v.y ); angular_velocity = v.z; }

	void setPosition( const Vec2& pos ) { position = pos; }
	void setVelocity( const Vec2& vel ) { if ( linear_enable ) { velocity = vel; } }
	void setLinearEnable( bool le ) { linear_enable = le; }

	void setAngle( const Scalar th ) { angle = th; }
	void setAngularVelocity( const Scalar w ) { if ( angular_enable ) { angular_velocity = w; } }
	void setAngularEnable( bool ae ) { angular_enable = ae; }

	void setLinearDamping( Scalar ld ) { linear_damping = ld; }
	void setAngularDamping( Scalar ad ) { angular_damping = ad; }

	void setGravity( Vec2 g ) { gravity = g; }

	void setMass( Scalar m ) { mass = m; }
	void setMoment( Scalar i ) { moment = i; }
	void setBounce( Scalar b ) { bounce = b; }
	void setFriction( Scalar k ) { friction = k; }

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
		mass,
		moment,
		bounce,
		friction;

private: // Members
	std::vector < Convex > shapes; // object space

private: // Physics engine graph data
	std::set < Constraint* > edges;
	bool marked;
	int local_id;
};

#endif
