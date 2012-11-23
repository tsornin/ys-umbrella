#ifndef PHYSICS_FRICTION_H
#define PHYSICS_FRICTION_H

#include "Constraint.h" // superclass Constraint

/*
================================
Friction constraint.

Represents a friction constraint.

NOTE: Friction constraints come and go with Contact constraints.
Therefore, unlike all other physics objects, some Friction objects
are owned by Contact objects.
================================
*/
class Friction : public Constraint
{
private: // Lifecycle
	Friction( Rigid* a, Rigid* b );
	Friction( const Friction& ) = delete;
	Friction& operator = ( const Friction& ) = delete;
	~Friction() = default;

	virtual void destroy( PhysicsState& ps );

public: // Constraint functions
	virtual Scalar eval() const;
	virtual std::pair < Vec3, Vec3 > jacobian() const;
	virtual Scalar bias( Scalar jv ) const;
	virtual std::pair < Scalar, Scalar > bounds() const;

	virtual void draw( Renderer& rd ) const {
		rd.drawFriction( *this );
	}

public: // Friction functions
	Scalar mix_friction() const;
	static Scalar mix_friction( Scalar k1, Scalar k2 );

public: // Members
	Vec2 tangent;
	Vec2 p;

	Scalar normal_lambda;

	friend class PhysicsState;
	friend class Contact;
};

#endif
