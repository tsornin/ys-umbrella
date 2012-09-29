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
public: // Constraint
	Friction( Rigid* a, Rigid* b );
public:
	virtual ~Friction() {}
	friend class PhysicsState;

	virtual Scalar eval(); // TODO: is this meaningful for friction?
	virtual std::pair < Vec3, Vec3 > jacobian();
	virtual Scalar bias( Scalar jv );
	virtual std::pair < Scalar, Scalar > bounds();

	virtual void draw( Renderer& rd ) { rd.drawFriction( *this ); }
	friend class Renderer;

public: // Members
	Vec2 tangent;
	Vec2 p;

	Scalar normal_lambda;
};

#endif
