#ifndef PHYSICS_RIGID_H
#define PHYSICS_RIGID_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include <vector>
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
class Rigid :
	public PhysicsTags,
	public PhysicsGraph < Rigid, Constraint >::Vertex
{
private: // Lifecycle
	Rigid();
	Rigid( const std::vector < Convex >& cs );
	Rigid( const Rigid& ) = delete;
	Rigid& operator = ( const Rigid& ) = delete;
	~Rigid() = default;

public: // "Entity" functions
	void input( const InputSet& is );
	void update();
	AABB getAABB() const;

public: // Rigid functions
	bool frozen() const { return !linear_enable && !angular_enable; }

	Vec2 world( const Vec2& p ) const;
	Vec2 local( const Vec2& p ) const;

	Vec2 getVelocityAt( const Vec2& p ) const;

	Vec3 getPositionState() const { return Vec3( position, angular_position ); }
	void setPositionState( const Vec3& p ) { position = Vec2( p.x, p.y ); angular_position = p.z; }

	Vec3 getVelocityState() const { return Vec3( velocity, angular_velocity ); }
	void setVelocityState( const Vec3& v ) { velocity = Vec2( v.x, v.y ); angular_velocity = v.z; }

	Vec3 getInverseMass() const {
		return Vec3(
			Vec2( linear_enable ? 1.0 / mass : 0 ),
			angular_enable ? 1.0 / moment : 0 );
	}

public: // Members
	// Position state
	Vec2
		position, // in 2D world-space
		velocity; // in units/frame
	bool linear_enable; // Must enable this to move.

	// Rotation state
	Scalar
		angular_position, // in radians
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

	friend class PhysicsState;
	template < typename T > friend struct Expire;
	friend class Renderer;
};

#endif
