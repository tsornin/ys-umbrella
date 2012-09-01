#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

#include "Constraint.h" // superclass Constraint

class Contact : public Constraint
{
public: // Constraint
	Contact( Rigid* a, Rigid* b );
	virtual ~Contact() {}

	virtual std::pair < Vec3, Vec3 > jacobian();

	// TODO:
	// virtual Scalar error();

	friend class PhysicsState;

public: // Contact specific
	Vec2 normal; // Points away from body A
	Scalar overlap; // Positive when penetrating
	Vec2 a_p;
	Vec2 b_p;
};

#endif
